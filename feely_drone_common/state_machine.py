import numpy as np
from enum import Enum
from .search_pattern import (LinearSearchPattern,
                             SinusoidalSearchPattern,
                             CompositeSearchPattern)
from .steady_state_calculator import get_contact_sensor_location, forward_kinematics

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
        self.contact_locs = forward_kinematics(p=np.zeros(4), 
                                               joint_angles=np.reshape([q0] * 3, [3,3]),
                                               p0=p0, rot0=rot0, l=l_arm)

        self.tau_min = -1
        

    def reset(self):

        self.t = 0
        self.target_pos_estimate = self.init_target_pos_estimate.copy()
        self.target_yaw_estimate = self.init_target_yaw_estimate.copy()
        self.alpha = np.ones(3)
        self.state = State.TAKEOFF  
        self.searching_pattern.reset()

        self.tactile_info_sw = np.zeros([10, 3, 3], dtype=float)
        
        self.reference_pos = np.zeros(3)
        self.contact_locs = forward_kinematics(p=np.zeros(4), 
                                               joint_angles=np.reshape([self.q0] * 3, [3,3]),
                                               p0=self.p0, rot0=self.rot0, l=self.l)

        self.tau_min = -1
        

    def set_takeoff_position(self, takeoff_pos):
        self.takeoff_position = takeoff_pos

    def update_target_pos_estimate(self, target_pos_estimate):
        self.init_target_pos_estimate = target_pos_estimate
        self.searching_pattern.params[3, :] = target_pos_estimate

    def update_target_yaw_estimate(self, target_yaw_estimate):
        self.init_target_yaw_estimate = target_yaw_estimate
    
    def get_des_yaw_vel(self, contacts, rot_vel=0.025):
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

        # Fully open the gripper
        self.alpha = np.ones(3)

        yaw_rate = np.sign(yaw_des - x[3]) * 0.1
        
        if np.linalg.norm(dist) < 0.25:
            v_des = 0.25 * np.append(dist, 0.0)
        elif np.linalg.norm(dist) < 0.1:
            v_des = 0.1 * np.append(dist, 0.0) 
        elif np.linalg.norm(dist) < 0.05:
            v_des = np.zeros(4)  
        else:
            v_des = 0.5 * np.append(dist, yaw_rate)

        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': v_des,
                'yaw_des': yaw_des}

    def searching_position_control(self, x, v, contact):

        # Explicitly state that we are not using this here
        _ = contact

        # Extract new desired control values
        p_des, v_des, self.tau_min = self.searching_pattern.get_ref_pos_vel(x=x[:3], last_tau=self.tau_min)

        # Handle the wraparound
        if self.tau_min >= 1.0:
            self.tau_min = 0.0

        # Append zero to v_des for yaw control
        v_des = np.append(v_des, 0.0)

        # Sinusoidal pattern in opening and closing the gripper being maximally open 
        # at the peaks of the searching pattern, i.e. tau = 0.125, 0.25, 0.375, 0.75
        self.alpha = (1 - (0.25 * np.sin(4 *  2 * np.pi * self.tau_min  + np.pi/2) + 0.25)) * np.ones(3)

        # If we're close to completing a full cycle
        if  self.tau_min > 0.99:   # tolerance band near 1
            self.searching_pattern.step_height(0.075)
            self.target_pos_estimate[2] += 0.075
            self.tau_min = -1.0
            # Reset init_tau so we don't immediately retrigger

            self.tau_min = -1.0

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

    def position_fine_alignment(self, contact, del_p=0.05):
        contact_yp = contact[0, :] | contact[2, :]
        contact_ym = contact[1, :]

        alignment = np.zeros(3)
        if np.sum(1 * contact_yp) > np.sum(1 * contact_ym):
            alignment = np.array([-del_p, 0,  del_p])
        elif np.sum(1 * contact_yp) < np.sum(1 * contact_ym):
            alignment = np.array([del_p, 0, del_p])
        return alignment
        

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
            
        if contact.any() and np.linalg.norm(self.target_pos_estimate[:2] - x[:2]) < 0.01:
            cT = np.cos(x[3])
            sT = np.sin(x[3])

            rot = np.array([[cT, -sT, 0],
                            [sT,  cT, 0],
                            [ 0,   0, 1]])
            delta_p = rot @ self.position_fine_alignment(contact)
            self.target_pos_estimate += delta_p
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

        self.alpha = np.zeros(3)
        v_des = np.zeros(4)
        v_des[3] = omega_des
        
        
        return {'alpha': self.alpha,
                'p_des': p_des,
                'v_des': v_des,
                'yaw_des': yaw_des}
    
    def get_new_ref_pos(self, x, contact):     

        cntc_sens_loc = get_contact_sensor_location(x[:4],
                                                    alpha=self.alpha,
                                                    M_g=self.M_g,
                                                    K=self.K,
                                                    A=self.A,
                                                    p0=self.p0,
                                                    rot0=self.rot0,
                                                    l=self.l)
        
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
            if np.linalg.norm(x[:3] - self.reference_pos) < 0.1:
                self.state = State.SEARCHING
                print("STATE CHANGE: TAKEOFF -> SEARCHING")
        elif self.state == State.SEARCHING:
            ctrl = self.searching_position_control(x, v, contact)
            self.reference_pos = ctrl["p_des"]  
            if contact.any():
                # Find new target pos estimate based on contact
                self.target_pos_estimate = self.get_new_ref_pos(x, contact)
                
                # Find difference in the xy-plane 
                planar_diff = self.target_pos_estimate - self.reference_pos

                # Move slightly away from the target
                self.reference_pos -= planar_diff / np.linalg.norm(planar_diff) * 0.1

                # Also move down
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
                self.reference_pos = self.target_pos_estimate
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
            # If each arm has at least one pad that has had
            # consistent contact ....
            if np.any(mean > 0.5, axis=1).all():
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
            # If on each arm any of the first pads
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
        self.contact_locs = get_contact_sensor_location(x[:4],
                                                    alpha=self.alpha,
                                                    M_g=self.M_g,
                                                    K=self.K,
                                                    A=self.A,
                                                    p0=self.p0,
                                                    rot0=self.rot0,
                                                    l=self.l)

        return ctrl
