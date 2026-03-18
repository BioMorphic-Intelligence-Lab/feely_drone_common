import numpy as np

gravity_vector = np.array([0, 0, -9.81])  # Gravity vector in the Z direction
q0 = np.deg2rad(75) * np.ones(3)



def rotation_matrix_from_euler(euler, degrees=False):
    """Create rotation matrix from Euler angles using numpy.

    Convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    
    Args:
        euler: Euler angles [roll, pitch, yaw] of shape (3,) or (batch, 3)
        degrees: If True, input angles are in degrees
        
    Returns:
        Rotation matrix of shape (3, 3) or (batch, 3, 3)
    """
    angles = np.asarray(euler)
    
    if degrees:
        angles = angles * (np.pi / 180.0)

    # Handle unbatched input
    if angles.ndim == 1 and angles.shape[0] == 3:
        batched = False
        angles = angles[None, :]
    else:
        batched = True

    roll = angles[..., 0]
    pitch = angles[..., 1]
    yaw = angles[..., 2]

    cx = np.cos(roll)
    sx = np.sin(roll)
    cy = np.cos(pitch)
    sy = np.sin(pitch)
    cz = np.cos(yaw)
    sz = np.sin(yaw)

    # Rotation matrices (batchable with broadcasting)
    Rx = np.zeros((angles.shape[0], 3, 3))
    Rx[:, 0, 0] = 1
    Rx[:, 1, 1] = cx
    Rx[:, 1, 2] = -sx
    Rx[:, 2, 1] = sx
    Rx[:, 2, 2] = cx

    Ry = np.zeros((angles.shape[0], 3, 3))
    Ry[:, 0, 0] = cy
    Ry[:, 0, 2] = sy
    Ry[:, 1, 1] = 1
    Ry[:, 2, 0] = -sy
    Ry[:, 2, 2] = cy

    Rz = np.zeros((angles.shape[0], 3, 3))
    Rz[:, 0, 0] = cz
    Rz[:, 0, 1] = -sz
    Rz[:, 1, 0] = sz
    Rz[:, 1, 1] = cz
    Rz[:, 2, 2] = 1

    # Combined rotation: R = Rz @ Ry @ Rx
    R = np.matmul(Rz, np.matmul(Ry, Rx))
    
    if not batched:
        return R[0]
    return R


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

    for _ in range(max_iter):
        tau = compute_gravity_tau(q, M_g)
        g_q = tau + K @ (q - q0) - A * tau_act

        if np.linalg.norm(g_q) < tol:
            return q

        J_g = compute_jacobian(q, M_g=M_g, K=K)
        if np.linalg.norm(np.linalg.det(J_g)) <= 1e-3:
            print("Configuration singularity at q =", q)

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
        p: np.array (6), xyz and roll-pitch-yaw [rad] position of base
        joint_angles: np.array (3,3), joint angles [arm, link].

    Returns:
        positions: np.array (3,3,3), containing XYZ positions of link centers.
    """
    positions = np.zeros((3, 3, 3))

    for i in range(3):  # Iterate over arms
        
        R_prev = rotation_matrix_from_euler(p[3:6]) # Start with base rotation
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
