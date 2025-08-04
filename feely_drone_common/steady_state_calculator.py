import numpy as np

gravity_vector = np.array([0, 0, -9.81])  # Gravity vector in the Z direction
q0 = np.deg2rad(75) * np.ones(3)


def compute_gravity_tau(q, M_g, g=gravity_vector):
        """ Compute gravity torques tau(q) """
        C = np.array([np.cos(q[0]), np.cos(q[0] + q[1]), np.cos(q[0] + q[1] + q[2])])
        return np.linalg.norm(g) * M_g @ C

def compute_jacobian(q, M_g, K, g=gravity_vector):
    """ Compute Jacobian J_g(q) """
    S = np.array([
        [np.sin(q[0]), np.sin(q[0] + q[1]), np.sin(q[0] + q[1] + q[2])],
        [0, np.sin(q[0] + q[1]), np.sin(q[0] + q[1] + q[2])],
        [0, 0, np.sin(q[0] + q[1] + q[2])]
    ])
    J_tau = -np.linalg.norm(g) * M_g @ S
    return J_tau + K

def newton_solve(tau_act, M_g, K, A, tol=1e-6, max_iter=100):
    """ Solve g(q) = 0 using Newton's method """
    q = np.zeros(3)

    for i in range(max_iter):
        tau = compute_gravity_tau(q, M_g)
        g_q = tau + K @ (q - q0) - A * tau_act

        if np.linalg.norm(g_q) < tol:
            return q

        J_g = compute_jacobian(q, M_g=M_g, K=K)
        delta_q = np.linalg.solve(J_g, -g_q)
        q += delta_q

    return q

def find_steady_state_config(alpha, M_g, K, A):
    config = newton_solve(alpha, M_g=M_g, K=K, A=A)
    return config

def forward_kinematics(p, joint_angles,
                       p0=None, rot0=None, l=None):
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
        p_prev = p[:3] + R_prev @ p0[i, :]   # Start with base position
        R_prev = R_prev @ rot0[i, :, :] # Add arm rotation
        
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
            p_current = p_prev + R_prev @ np.array([0, 0, l[j]])
            
            # Store the position
            positions[i, j] = p_current #- R_prev @ np.array([self.l[j] / 2, 0, 0])
            
            # Update for next iteration
            R_prev = R_current
            p_prev = p_current

    return positions

def get_contact_sensor_location(p, alpha,
                                M_g, K, A,
                                p0, rot0, l):

    config = find_steady_state_config(alpha, M_g=M_g, K=K, A=A)
    config = np.reshape([config] * 3, [3,3])
    locs = forward_kinematics(p, config,
                              p0=p0, rot0=rot0, l=l)

    return locs
