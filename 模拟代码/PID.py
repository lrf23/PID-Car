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
    def restart(self):
        self.ep = 0.0
        self.ei = 0.0
        self.ed = 0.0
# 增量式,可以无视
class PID_inc:
    """增量式实现
    """
    def __init__(self, k, upper=np.pi/4, lower=-np.pi/4):
        self.kp, self.ki, self.kd = k   
        self.err = 0
        self.err_last = 0
        self.err_ll = 0

        self.upper = upper
        self.lower = lower
        self.value = 0
        self.inc = 0

    def cal_output(self, error):
        self.err = error
        self.inc = self.kp * (self.err - self.err_last) + self.ki * self.err + self.kd * (
            self.err - 2 * self.err_last + self.err_ll)
        self._update()
        return self.value

    def _update(self):
        self.err_ll = self.err_last
        self.err_last = self.err
        self.value = self.value + self.inc
        if self.value > self.upper:
            self.value = self.upper
        elif self.value < self.lower:
            self.value = self.lower

    def set_k(self, k):
        self.kp, self.ki, self.kd = k

    def set_bound(self, upper, lower):
        self.upper_bound = upper
        self.lower_bound = lower

