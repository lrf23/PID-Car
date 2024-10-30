import numpy as np
def normalize_angle(angle):
    """将角度归一化到[-pi,pi]

    Args:
        angle (_type_): 输入角度

    Returns:
        _type_: 归一化后的角度
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle
class CarPath():
    def __init__(self,dt=0.01,interval=[0,2],type=0):
        """使用了0-2*pi的角度表示方式"""
        # set reference trajectory
        # refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k
        self.dt=dt#步长
        self.num=int((interval[1]-interval[0])/dt)#路径点个数
        self.interval=interval
        self.type=type
        self.refer_path = np.zeros((self.num, 4))
        self.refer_path[:, 0] = np.linspace(self.interval[0], self.interval[1], self.num)  # x
        if (self.type==0):
            self.refer_path[:,1]=0.8*np.sin(self.refer_path[:,0])
        else:
            self.refer_path[:,1]=0.4*self.refer_path[:,0]
        #self.refer_path[:, 1] = 10*np.sin(self.refer_path[:, 0]*0.3)  # y
        # 使用差分的方式计算路径点的一阶导和二阶导，从而得到切线方向和曲率
        for i in range(len(self.refer_path)):
            if i == 0:
                dx = self.refer_path[i+1, 0] - self.refer_path[i, 0]
                dy = self.refer_path[i+1, 1] - self.refer_path[i, 1]
                ddx = self.refer_path[2, 0] + \
                    self.refer_path[0, 0] - 2*self.refer_path[1, 0]
                ddy = self.refer_path[2, 1] + \
                    self.refer_path[0, 1] - 2*self.refer_path[1, 1]
            elif i == (len(self.refer_path)-1):
                dx = self.refer_path[i, 0] - self.refer_path[i-1, 0]
                dy = self.refer_path[i, 1] - self.refer_path[i-1, 1]
                ddx = self.refer_path[i, 0] + \
                    self.refer_path[i-2, 0] - 2*self.refer_path[i-1, 0]
                ddy = self.refer_path[i, 1] + \
                    self.refer_path[i-2, 1] - 2*self.refer_path[i-1, 1]
            else:
                dx = self.refer_path[i+1, 0] - self.refer_path[i, 0]
                dy = self.refer_path[i+1, 1] - self.refer_path[i, 1]
                ddx = self.refer_path[i+1, 0] + \
                    self.refer_path[i-1, 0] - 2*self.refer_path[i, 0]
                ddy = self.refer_path[i+1, 1] + \
                    self.refer_path[i-1, 1] - 2*self.refer_path[i, 1]
            self.refer_path[i, 2] = np.arctan2(dy, dx)  # yaw,[-pi,pi]
            if (self.refer_path[i, 2] < 0):
                self.refer_path[i, 2] += 2*np.pi
            # if (self.refer_path[i, 2] < 0):
            #     self.refer_path[i, 2] += 2*np.pi
            # 计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
            # 参考：https://blog.csdn.net/weixin_46627433/article/details/123403726
            self.refer_path[i, 3] = (
                ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))  # 曲率k计算

    