import numpy as np
from enum import Enum
from .search_pattern import (LinearSearchPattern,
                             SinusoidalSearchPattern,
                             CompositeSearchPattern)

class State(Enum):
    UNDEFINED = 0
    SEARCHING = 1
    TOUCHED = 2
    APPROACH = 3
    POSITION = 4
    ROTATION = 5
    FINALIZE = 6
    PERCH = 7
    ABORT = 8
    TAKEOFF = 9

class StateMachine(object):

    def __init__(self,
                 dt, m_arm, l_arm, K, A, g, q0, 
                 p0, rot0,
                 searching_pattern=None,
                 target_pos_estimate=np.array([0, 0, 0.5]),
                 target_yaw_estimate=np.zeros(1),
                 alpha_rate=1.0/10.0,
                 takeoff_position=np.array([0, 0, 1.5])):

        if searching_pattern is None:
            self.searching_pattern = (CompositeSearchPattern([
                    SinusoidalSearchPattern(params=np.stack([[0.75, 0.75, 0], # Amplitude
                                                     [2.0, 1.0, 0.0], # Frequency
                                                     [0.0, 0.0, 0.0], # Phase Shift
                                                     target_pos_estimate]) # Offset
                                        )
                    ])
            )
        else:
            self.searching_pattern = searching_pattern

        self.dt = dt
        self.t = 0
        self.init_target_pos_estimate = target_pos_estimate.copy()
        self.init_target_yaw_estimate = target_yaw_estimate.copy()
        self.target_yaw_estimate = target_yaw_estimate
        self.target_pos_estimate = target_pos_estimate
        self.alpha_rate = alpha_rate
        self.alpha = np.ones(3)
        self.state = State.TAKEOFF
        self.takeoff_position = takeoff_position   
        
        self.tactile_info_sw = np.zeros([10, 3, 3], dtype=float)

        self.p0 = p0
        self.rot0 = rot0
        self.K = K
        self.q0 = q0
        self.A = A
        self.g = g
        self.l = l_arm
        self.M_g = np.array([
            [(np.sum(m_arm)*0.5*l_arm[0]), np.sum(m_arm[1:])*0.5*l_arm[1], m_arm[2]*0.5*l_arm[2]],
            [                         0.0, np.sum(m_arm[1:])*0.5*l_arm[1], m_arm[2]*0.5*l_arm[2]],
            [                         0.0,                            0.0, m_arm[2]*0.5*l_arm[2]]
        ])

        self.reference_pos = np.zeros(3)
        self.contact_locs = self.forward_kinematics(np.zeros(4), np.reshape([q0] * 3, [3,3]))

        self.tau_min=-1

    def reset(self):

        self.t = 0
        self.target_pos_estimate = self.init_target_pos_estimate.copy()
        self.target_yaw_estimate = self.init_target_yaw_estimate.copy()
        self.alpha = np.ones(3)
        self.state = State.TAKEOFF  
        self.searching_pattern.reset()

        self.tactile_info_sw = np.zeros([10, 3, 3], dtype=float)
        
        self.reference_pos = np.zeros(3)
        self.contact_locs = self.forward_kinematics(np.zeros(4), np.reshape([self.q0] * 3, [3,3]))

        self.tau_min=-1

    def get_des_yaw_vel(self, contacts, rot_vel=0.2):
        rows = np.sum(np.array([1.0, 3.0, 2.0]) * contacts, axis=1)
        #if rows[0] == rows[2]:
        #    return 0.0
        #elif rows[0] > rows[2]:
        #    return -rot_vel
        #elif rows[2] > rows[0]:
        #    return rot_vel
        #else:
        #    return 0.0
        if rows[1] == rows[0]:
            return 0.0
        elif rows[1] > rows[0]:
            return -rot_vel
        elif rows[0] > rows[1]:
            return rot_vel
        else:
            return 0.0

    def takeoff_control(self, x, v, contact):
        """
        Control for takeoff state.
        This is a placeholder and should be implemented based on the specific requirements of the takeoff procedure.
        """
        yaw_des = self.target_yaw_estimate
        p_des = self.takeoff_position
        dist = self.takeoff_position - x[:3]
        
        if np.linalg.norm(dist) < 0.25:
            v_des = np.zeros(4)  # Stop when close enough
        else:
            v_des = 0.5 * np.append(dist, 0)  # No yawrate control during takeoff

        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': v_des,
                'yaw_des': yaw_des}

    def searching_position_control(self, x, v, contact):

        # Explicitly state that we are not using this here
        _ = contact

        # Extract new desired control values
        p_des, v_des, self.tau_min = self.searching_pattern.get_ref_pos_vel(x=x[:3], last_tau=self.tau_min)

        # Append zero to v_des for yaw control
        v_des = np.append(v_des, 0.0)

        # Sinusoidal pattern in opening and closing the gripper
        self.alpha = (1 - (0.25 * np.sin(2*np.pi / 10.0 * self.t) + 0.25)) * np.ones(3)

        if self.tau_min >= 0.99:
            self.searching_pattern.step_height(0.075)
            self.target_pos_estimate[2] += 0.1
            self.tau_min = -1

        yaw_des = self.target_yaw_estimate
        
        # Return control actions
        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': v_des,
                'yaw_des': yaw_des}

    def position_align_control(self, x, v, contact, p_des=None):
        
        
        if p_des is None:
            p_des = self.target_pos_estimate
        yaw_des = self.target_yaw_estimate

        v_des = np.zeros(4)        
        
        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': v_des,
                'yaw_des': yaw_des}
    
    def abort_control(self, x, v, contact):
        
        
        p_des = self.target_pos_estimate
        yaw_des = self.target_yaw_estimate
        
        self.alpha = np.ones(3)
        v_des = np.zeros(4)
        
        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': v_des,
                'yaw_des': yaw_des}

    def position_fine_alignment(self, contact, del_p=0.01):
        contact_yp = contact[0, :] | contact[2, :]
        contact_ym = contact[1, :]

        if np.sum(1 * contact_yp) > np.sum(1 * contact_ym):
            return np.array([0, del_p, 0])
        elif np.sum(1 * contact_yp) < np.sum(1 * contact_ym):
            return np.array([0, -del_p, 0])
        else:
            return np.zeros(3)

    def rotation_align_control(self, x, v, contact):
        
        omega_des = self.get_des_yaw_vel(contact)
        self.target_yaw_estimate += self.dt * omega_des
        
        yaw_des = self.target_yaw_estimate

        cT = np.cos(x[3])
        sT = np.sin(x[3])

        rot = np.array([[cT, -sT, 0],
                        [sT,  cT, 0],
                        [ 0,   0, 1]])
        
        if contact.any() and np.linalg.norm(self.target_pos_estimate[:2] - x[:2]) < 0.01:
            delta_p = rot @ self.position_fine_alignment(contact)
            self.target_pos_estimate += delta_p
        
        p_des = self.target_pos_estimate

        dalpha = -self.alpha_rate * self.dt * np.ones(3)
        indeces = ~contact.any(axis=1)
        self.alpha[indeces] += dalpha[indeces]
        self.alpha = np.clip(self.alpha, a_min=0.0, a_max=1.0)
        
        v_des = np.zeros(4)
        v_des[3] = omega_des

        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': v_des,
                'yaw_des': yaw_des}

    def finalize_grasp_control(self, x, v, contact):
            
        p_des = self.target_pos_estimate
        yaw_des = self.target_yaw_estimate

        self.alpha = np.ones(3) * np.min(self.alpha)
        v_des = np.zeros(4)
        
        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': v_des,
                'yaw_des': yaw_des}
    
    def perch_control(self, x, v, contact):

        p_des = self.target_pos_estimate
        omega_des = self.get_des_yaw_vel(contact, rot_vel=0.01)
        self.target_yaw_estimate += self.dt * omega_des
        yaw_des = self.target_yaw_estimate

        self.alpha = np.ones(3) * np.min(self.alpha)
        v_des = np.zeros(4)
        v_des[3] = omega_des
        
        
        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': v_des,
                'yaw_des': yaw_des}
    
    def compute_gravity_tau(self, q):
        """ Compute gravity torques tau(q) """
        C = np.array([np.cos(q[0]), np.cos(q[0] + q[1]), np.cos(q[0] + q[1] + q[2])])
        return np.linalg.norm(self.g) * self.M_g @ C

    def compute_jacobian(self, q):
        """ Compute Jacobian J_g(q) """
        S = np.array([
            [np.sin(q[0]), np.sin(q[0] + q[1]), np.sin(q[0] + q[1] + q[2])],
            [0, np.sin(q[0] + q[1]), np.sin(q[0] + q[1] + q[2])],
            [0, 0, np.sin(q[0] + q[1] + q[2])]
        ])
        J_tau = -np.linalg.norm(self.g) * self.M_g @ S
        return J_tau + self.K

    def newton_solve(self, tau_act, tol=1e-6, max_iter=100):
        """ Solve g(q) = 0 using Newton's method """
        q = np.zeros(3)

        for i in range(max_iter):
            tau = self.compute_gravity_tau(q)
            g_q = tau + self.K @ (q - self.q0) - self.A * tau_act

            if np.linalg.norm(g_q) < tol:
                return q

            J_g = self.compute_jacobian(q)
            delta_q = np.linalg.solve(J_g, -g_q)
            q += delta_q

        return q
    
    def find_steady_state_config(self, alpha):
        config = self.newton_solve(alpha)
        return config

    def forward_kinematics(self, p, joint_angles):
        """
        Computes the forward kinematics for a system with 3 arms, each with 3 links.
        
        Args:
           p: np.array (4), xyz and yaw [rad] position of base
           joint_angles: np.array (3,3), joint angles [arm, link].

        Returns:
            positions: np.array (3,3,3), containing XYZ positions of link centers.
        """
        positions = np.zeros((3, 3, 3))

        for i in range(3):  # Iterate over arms
            
            cT, sT = np.cos(p[3]), np.sin(p[3])
            R_prev = np.array([
                [cT, -sT, 0],
                [sT,  cT, 0],
                [0,    0, 1]
            ])               # Start with base rotation
            p_prev = p[:3] + R_prev @ self.p0[i, :]   # Start with base position
            R_prev = R_prev @ self.rot0[i, :, :] # Add arm rotation
            
            for j in range(3):  # Iterate over links
                # Joint rotation (assuming rotation around Z-axis for simplicity)
                cT, sT = np.cos(joint_angles[i, j]), np.sin(joint_angles[i, j])
                R_joint = np.array([
                    [1,  0,   0],
                    [0, cT, -sT],
                    [0, sT,  cT]
                ])
                # Compute current rotation
                R_current = R_prev @ R_joint
                
                # Compute current position
                p_current = p_prev + R_prev @ np.array([0, 0, self.l[j]])
                
                # Store the position
                positions[i, j] = p_current #- R_prev @ np.array([self.l[j] / 2, 0, 0])
                
                # Update for next iteration
                R_prev = R_current
                p_prev = p_current

        return positions

    def get_contact_sensor_location(self, p):

        config = self.find_steady_state_config(self.alpha)
        config = np.reshape([config] * 3, [3,3])
        locs = self.forward_kinematics(p, config)

        return locs

    def get_new_ref_pos(self, x, contact):     

        cntc_sens_loc = self.get_contact_sensor_location(x[:4])
        
        ref_pos = np.zeros(3, dtype=float)
        cntc_pts = 0.0
        for i in range(len(contact)):
            for j in range(len(contact[i])):
                if contact[i, j]:
                    cntc_pts += 1.0
                    ref_pos += cntc_sens_loc[i, j, :] 

        return ref_pos / cntc_pts   

    def update_tactile_info_sw(self, contact):
        self.tactile_info_sw = np.roll(self.tactile_info_sw, shift=-1, axis=0)
        self.tactile_info_sw[-1] = contact

        return np.mean(1.0 * self.tactile_info_sw, axis=0)

    def control(self, x, v, contact):

        if self.state == State.TAKEOFF:
            ctrl = self.takeoff_control(x, v, contact)
            self.reference_pos = ctrl["p_des"]  
            if np.linalg.norm(x[:3] - self.reference_pos) < 0.05:
                self.state = State.SEARCHING
                print("STATE CHANGE: TAKEOFF -> SEARCHING")
        elif self.state == State.SEARCHING:
            ctrl = self.searching_position_control(x, v, contact)
            self.reference_pos = ctrl["p_des"]  
            if contact.any():
                self.target_pos_estimate = self.get_new_ref_pos(x, contact) - np.array([0, 0, 0.1])
                self.reference_pos -= np.array([0.0, 0.0, 0.15])
                self.state = State.TOUCHED
                print("STATE CHANGE: SEARCHING -> TOUCHED")
        elif self.state == State.TOUCHED:
            # Fully open the gripper if the touch happened
            self.alpha = np.ones(3)
            ctrl = self.position_align_control(x, v, contact, self.reference_pos)
            if np.linalg.norm(x[:3] - self.reference_pos) < 0.05:
                self.state = State.APPROACH
                self.reference_pos = self.target_pos_estimate - np.array([0, 0, 0.2])
                print("STATE CHANGE: TOUCHED -> APPROACH")
        elif self.state == State.APPROACH:
            ctrl = self.position_align_control(x, v, contact, self.reference_pos)
            if np.linalg.norm(x[:3] - self.reference_pos) < 0.05:
                self.state = State.POSITION
                self.reference_pos = self.target_pos_estimate
                print("STATE CHANGE: APPROACH -> POSITION")
        elif self.state == State.POSITION:
            ctrl = self.position_align_control(x, v, contact)
            self.reference_pos = ctrl["p_des"]
            if np.linalg.norm(x[:3] - self.target_pos_estimate) < 0.1:
                self.state = State.ROTATION
                print("STATE CHANGE: POSITION -> ROTATION")
            # Check for ABORT condition
            if np.linalg.norm(x[:3] - self.reference_pos) > 1.5:
                self.target_pos_estimate += np.array([0, 0, -0.25])
                self.state = State.ABORT
                print("STATE CHANGE: POSITION -> ABORT")
        elif self.state == State.ROTATION:
            ctrl = self.rotation_align_control(x, v, contact)
            mean = self.update_tactile_info_sw(contact)
            # If each arm has at leat one pad that has had
            # consistent contact ....
            if np.any(mean > 0.9, axis=1).all():
                self.state = State.FINALIZE
                print("STATE CHANGE: ROTATION -> FINALIZE")
            # Check for ABORT condition
            if np.linalg.norm(x[:3] - self.reference_pos) > 1.5:
                self.target_pos_estimate += np.array([0, 0, -0.25])
                self.state = State.ABORT
                print("STATE CHANGE: ROTATION -> ABORT")
        elif self.state == State.FINALIZE:
            ctrl = self.finalize_grasp_control(x, v, contact)
            mean = self.update_tactile_info_sw(contact)
            # If on each arm any of the firsst pads
            #  have had consitent contact ...
            if (np.any(mean[:, :2] > 0.95, axis=1).all()
                # ... and all of the tendon tensions have equalized
                or np.allclose(self.alpha, max(self.alpha), atol=1)
                ):
                self.state = State.PERCH
                print("STATE CHANGE: FINALIZE -> PERCH")
            # Check for ABORT condition
            if np.linalg.norm(x[:3] - self.reference_pos) > 1.5:
                self.target_pos_estimate += np.array([0, 0, -0.25])
                self.state = State.ABORT
                print("STATE CHANGE: FINALIZE -> ABORT")
        elif self.state == State.PERCH:
            ctrl = self.perch_control(x, v, contact)
            mean = self.update_tactile_info_sw(contact)
        elif self.state == State.ABORT:
            ctrl = self.abort_control(x, v, contact)
            if np.linalg.norm(x[:3] - self.target_pos_estimate) < 0.05:
                self.state = State.SEARCHING
                print("STATE CHANGE: ABORT -> SEARCHING")
        else:
            ctrl = self.position_align_control(x, v, contact)
    
        self.t += self.dt
        self.contact_locs = self.get_contact_sensor_location(x[:4])

        return ctrl
