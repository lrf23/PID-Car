import copy
import numpy as np
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_w=np.pi/4
        self.ep = 0.0
        self.ei = 0.0
        self.ed = 0.0
        self.dt = 0.1
    def update_e(self, e):
        #print(e)
        self.ed = e - self.ep
        self.ei += e
        self.ep = copy.deepcopy(e)
        
    def get_u(self):
        u = self.kp*self.ep+self.ki*self.ei+self.kd*self.ed
        if u > self.max_w: u = self.max_w       
        if u < -self.max_w: u = -self.max_w
        #print(u)
        return u



