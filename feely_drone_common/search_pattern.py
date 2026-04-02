import numpy as np
from abc import ABC, abstractmethod

class SearchPattern(ABC):
    """
    Abstract base class for defining search patterns.
    """

    def __init__(self, params, n=500, vel_norm=1.0, dt=0.01):
        assert params.shape[1] == 3, "Parameters need to be three dimensional!"
        # Save the parameters
        self.init_params = params.copy()
        self.params = params.copy()
        # Create a discrete set of sampling points along tau
        self.tau_dis = np.linspace(0, 1, n)
        # Find the corresponding trajectory points
        self.traj_dis = np.array([self.f(tau) for tau in self.tau_dis])
        # Save the velocity norm and dt
        self.vel_norm = vel_norm
        self.dt=dt

    @abstractmethod
    def f(self, tau):
        """
        Compute the position at point tau along the search pattern.
        """
        pass

    @abstractmethod
    def df(self, tau):
        """
        Compute the velocity at point tau along the search pattern.
        """
        pass

    @abstractmethod
    def ddf(self, tau):
        """
        Compute the velocity at point tau along the search pattern.
        """
        pass

    def reset(self):
        """
        Reset the search pattern to its initial state.
        """
        self.params = self.init_params.copy()

        # Find the corresponding trajectory points
        self.traj_dis = np.array([self.f(tau) for tau in self.tau_dis])

    def step_height(self, dh):
        """
        Add a step height to the search pattern.
        """
        # Update height of search pattern
        self.params[-1, 2] += dh

        # Find the corresponding trajectory points
        self.traj_dis = np.array([self.f(tau) for tau in self.tau_dis])

    def find_nearest_tau(self, x, last_tau=-1, iter=2):
        """
        Find the tau \\in [0,1] that correspondes to the point p with the 
        minimal distance to the queried point x
        """
        # If a last tau is given we start the newton's method refinement
        # from that last tau
        if last_tau >= 0:
            tau = last_tau
        # Otherwise we do a brute search over the discrete traj points
        else:
            # Find the vector of squared distances from the current x to the 
            # discrete samples of the trajectory
            distances2 = ((self.traj_dis[0, :] - x[0])**2
                        + (self.traj_dis[1, :] - x[1])**2
                        + (self.traj_dis[2, :] - x[2])**2)
            # Find the best tau
            tau = self.tau_dis[np.argmin(distances2)]
        
        # Define the functions for the newton optimization step
        dg = lambda t: 2*(np.sum((self.f(t) - x) * self.df(t)))
        ddg = lambda t: 2*(np.sum(self.df(t)**2 + (self.f(t) - x) * self.ddf(t)))

        # Run the newton optimization
        for _ in range(iter):
           tau = tau - dg(tau) / ddg(tau)

        # Cap the return value between 0 and 1 and return
        return np.clip(tau, a_min=0, a_max=1)

    def field(self, x, last_tau=-1, kappa=50.0):

        # Find minimum distance trajectory point
        t_min = self.find_nearest_tau(x, last_tau=last_tau)

        # Now compute the normalized distance vector
        distance_vector = (self.f(t_min).flatten() - x)
        distance_norm = np.linalg.norm(distance_vector)

        # Find traj derivative at that index
        derivative = self.df(t_min).flatten()

        # Compute the gradient
        gradient = (np.exp(-distance_norm) * derivative
                    + kappa * distance_vector)

        # Normalize if exceeding desired velocity and stretch according to velocity
        norm = np.linalg.norm(gradient)
        if norm > self.vel_norm:
            return (gradient / norm * self.vel_norm, t_min)
        else:
            return (gradient, t_min)

    def get_ref_pos_vel(self, x, last_tau=-1):
        """
        Get the reference position and velocity for point x in space.
        """

        # Find the vector field value at the current position
        gradient, tau_min = self.field(x, last_tau=last_tau)

        # Find the corresponding position and velocity
        p_des = x + self.dt * gradient
        v_des = gradient

        return p_des, v_des, tau_min

class LinearSearchPattern(SearchPattern):
    def __init__(self, params, vel_norm=1.0, dt=0.01):
        super().__init__(params, vel_norm=vel_norm, dt=dt)
        # params[0, :] Slope; params[1, :] Offset

    def f(self, tau):
        return self.params[1, :] + self.params[0, :] * tau

    def df(self, tau):
        return self.params[0, :]
    
    def ddf(self, tau):
        return 0

    def flip_direction(self):
        """Reverse the traversal direction so that the old endpoint becomes the
        new start point (tau=0). After this call the drone, which is near f(1)
        of the previous pass, will project to tau≈0 on the new path, allowing
        find_nearest_tau to restart naturally from the beginning."""
        # Move the offset to the old end-point: new f(0) = old f(1)
        self.params[1, :] = self.params[1, :] + self.params[0, :]
        # Negate the slope so the path is traversed in the opposite direction
        self.params[0, :] = -self.params[0, :]
        self.traj_dis = np.array([self.f(tau) for tau in self.tau_dis])

class SinusoidalSearchPattern(SearchPattern):
    def __init__(self, params, vel_norm=1.0, dt=0.01):
        super().__init__(params, vel_norm=vel_norm, dt=dt)

    def f(self, tau):
        return (self.params[3, :]
                + self.params[0, :] * np.sin(2 * np.pi * self.params[1, :] * tau
                                          + self.params[2, :]))

    def df(self, tau):
        return (self.params[0, :] * self.params[1, :] * 2 * np.pi
                * np.cos(2 * np.pi * self.params[1, :] * tau
                                          + self.params[2, :]))
    def ddf(self, tau):
        return (-self.params[0, :] * (self.params[1, :] * 2 * np.pi)**2
                * np.sin(2 * np.pi * self.params[1, :] * tau
                                          + self.params[2, :]))

class SquareSearchPattern(SearchPattern):
    def __init__(self, params, vel_norm=1.0, dt=0.01):
        super().__init__(params, vel_norm=vel_norm, dt=dt)
        # params[0, 0] Side length, params[1, :] Centerpoint

    def smooth_trans(self, vel_scale, v_from, v_to, t):
        """Smooth interpolation between directions at the corner: t in [0, 1]."""
        # Use cosine to get smooth directional blend (cosine interpolation)
        return vel_scale * (
            (1 - 0.5 * (1 - np.cos(np.pi * t))) * v_from +
            (0.5 * (1 - np.cos(np.pi * t))) * v_to
        )

    def f(self, tau):
        # This produces a square path, with tau in [0, 1), each side is 0.25 long in tau
        # params[0, 0]: side length; params[1, :]: center of square (3D)
        side = self.params[0, 0]
        center = self.params[1, :]
        tau_local = tau % 1.0

        if tau_local < 0.25:
            # Side 1: left to right (bottom)
            t = (tau_local - 0.0) / 0.25   # [0,1)
            p0 = center + 0.5 * side * np.array([-1, -1, 0])  # Bottom-left
            p1 = center + 0.5 * side * np.array([ 1, -1, 0])  # Bottom-right
            pos = p0 + (p1 - p0) * t
        elif tau_local < 0.5:
            # Side 2: bottom to top (right)
            t = (tau_local - 0.25) / 0.25
            p0 = center + 0.5 * side * np.array([ 1, -1, 0])  # Bottom-right
            p1 = center + 0.5 * side * np.array([ 1,  1, 0])  # Top-right
            pos = p0 + (p1 - p0) * t
        elif tau_local < 0.75:
            # Side 3: right to left (top)
            t = (tau_local - 0.5) / 0.25
            p0 = center + 0.5 * side * np.array([ 1,  1, 0])  # Top-right
            p1 = center + 0.5 * side * np.array([-1,  1, 0])  # Top-left
            pos = p0 + (p1 - p0) * t
        else:
            # Side 4: top to bottom (left)
            t = (tau_local - 0.75) / 0.25
            p0 = center + 0.5 * side * np.array([-1,  1, 0])  # Top-left
            p1 = center + 0.5 * side * np.array([-1, -1, 0])  # Bottom-left
            pos = p0 + (p1 - p0) * t

        return pos

    def df(self, tau):
        tau_local = tau % 1.0
        side = self.params[0, 0]
        corner_eps = 0.04  # smoothing window at each corner, can be tuned

        # Main velocities for straight edges
        v1 = np.array([1, 0, 0])   # bottom: left to right
        v2 = np.array([0, 1, 0])   # right: bottom to top
        v3 = np.array([-1, 0, 0])  # top: right to left
        v4 = np.array([0, -1, 0])  # left: top to bottom
        vel_scale = side / 0.25

        if tau_local < 0.25 - corner_eps:
            vel = vel_scale * v1
        elif tau_local < 0.25 + corner_eps:
            # Transition: v1 → v2
            t = (tau_local - (0.25 - corner_eps)) / (2 * corner_eps)
            vel = self.smooth_trans(vel_scale, v1, v2, t)
        elif tau_local < 0.5 - corner_eps:
            vel = vel_scale * v2
        elif tau_local < 0.5 + corner_eps:
            # Transition: v2 → v3
            t = (tau_local - (0.5 - corner_eps)) / (2 * corner_eps)
            vel = self.smooth_trans(vel_scale, v2, v3, t)
        elif tau_local < 0.75 - corner_eps:
            vel = vel_scale * v3
        elif tau_local < 0.75 + corner_eps:
            # Transition: v3 → v4
            t = (tau_local - (0.75 - corner_eps)) / (2 * corner_eps)
            vel = self.smooth_trans(vel_scale, v3, v4, t)
        else:
            if tau_local < 1.0 - corner_eps:
                vel = vel_scale * v4
            else:
                # Transition: v4 → v1 (wrap-around)
                t = (tau_local - (1.0 - corner_eps)) / (2 * corner_eps)
                # For wrap-around, interpolate v4 to v1
                vel = self.smooth_trans(vel_scale, v4, v1, t)
        return vel
    
    def ddf(self, tau):
        return np.zeros(3)

class SpiralSearchPattern(SearchPattern):
    def __init__(self, params, vel_norm=1.0, dt=0.01):
        super().__init__(params, vel_norm=vel_norm, dt=dt)
        # params[0, 0] Max Radius; params[0, 1] Rotation Speed; params[1, :] Centerpoint

    def f(self, tau):
        return (
            self.params[1,:] + self.params[0, 0] * tau * 
            np.array([np.cos(2 * np.pi * self.params[0, 1] * tau),
                      np.sin(2 * np.pi * self.params[0, 1] * tau), 0])
            )

    def df(self, tau):
        return (
            self.params[0, 0] * 
            np.array([np.cos(2 * np.pi * self.params[0, 1] * tau) - 2 * np.pi * self.params[0, 1] * tau * np.sin(2 * np.pi * self.params[0, 1] * tau),
                      np.sin(2 * np.pi * self.params[0, 1] * tau) + 2 * np.pi * self.params[0, 1] * tau * np.cos(2 * np.pi * self.params[0, 1] * tau),
                      0])
        )

    def ddf(self, tau):
        return (4 * np.pi * self.params[0, 0] * self.params[0, 1] *
            np.array([-np.sin(2 * np.pi * tau * self.params[0, 1]) - np.pi * tau * self.params[0, 1] * np.cos(2 * np.pi * tau * self.params[0, 1]),
                       np.cos(2 * np.pi * tau * self.params[0, 1]) - np.pi * tau * self.params[0, 1] * np.sin(2 * np.pi * tau * self.params[0, 1]),
                      0])

        )

class CompositeSearchPattern(SearchPattern):
    def __init__(self, patterns, vel_norm=1.0, dt=0.01):
        self.patterns = patterns
        super().__init__(np.zeros([1, 3]), vel_norm=vel_norm, dt=dt)

    def f(self, tau):
        pos = np.zeros(3)
        for pattern in self.patterns:
            pos += pattern.f(tau)
        return pos

    def df(self, tau):
        vel = np.zeros(3)
        for pattern in self.patterns:
            vel += pattern.df(tau)
        return vel
    
    def ddf(self, tau):
        acc = np.zeros(3)
        for pattern in self.patterns:
            acc += pattern.ddf(tau)
        return acc
    
    def step_height(self, dh):
        for pattern in self.patterns:
            pattern.step_height(dh / len(self.patterns))

        # Find the corresponding trajectory points
        self.traj_dis = np.array([self.f(tau) for tau in self.tau_dis])

    def reset(self):
        """
        Reset the search pattern to its initial state.
        """
        for pattern in self.patterns:
            pattern.params = pattern.init_params

        # Find the corresponding trajectory points
        self.traj_dis = np.array([self.f(tau) for tau in self.tau_dis])