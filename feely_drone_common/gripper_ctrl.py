import numpy as np

class GripperCtrl:

    def __init__(self, tau_max, p_gain=0.01, d_gain=0.1):
        
        # Remember maximum force
        self.tau_max = tau_max

        # Current alpha
        self.alpha = 0.0
        self.dalpha = 0.0
        
        # Gains
        self.p_gain = p_gain
        self.d_gain = d_gain

    def reset(self):
        # Current alpha
        self.alpha = 0.0
        self.dalpha = 0.0

    def open_to(self, alpha_des):
        
        assert (0 <= alpha_des).all() and (alpha_des <= 1).all(), f"Commanded opening state ({alpha_des}) outside allowed range [0, 1]"

        err = (alpha_des - self.alpha)
        alpha_new = np.clip(self.alpha + self.p_gain * err - self.d_gain * self.dalpha, 0, 1) 
        self.dalpha = (alpha_new - self.alpha)
        self.alpha = alpha_new

        return  self.alpha*self.tau_max