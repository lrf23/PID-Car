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
class MyPath:
    def __init__(self,dt=0.1,interval=[0,100],type=0):
        """使用了-pi-pi的角度表示方式"""
        # set reference trajectory
        # refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k
        self.dt=dt#步长
        self.num=int((interval[1]-interval[0])/dt)#路径点个数
        self.interval=interval
        self.type=type
        self.refer_path = np.zeros((self.num, 4))
        self.refer_path[:, 0] = np.linspace(self.interval[0], self.interval[1], self.num)  # x
        if (self.type==0):
            self.refer_path[:, 1] = 2*np.sin(self.refer_path[:, 0]/3.0) + \
                2.5*np.cos(self.refer_path[:, 0]/2.0)  # y
        else:
            self.refer_path[:, 1] = 10*np.sin(self.refer_path[:, 0]*0.3)
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
            # if (self.refer_path[i, 2] < 0):
            #     self.refer_path[i, 2] += 2*np.pi
            # 计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
            # 参考：https://blog.csdn.net/weixin_46627433/article/details/123403726
            self.refer_path[i, 3] = (
                ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))  # 曲率k计算

    def calc_track_error(self, x, y):
        """计算跟踪误差

        Args:
            x (_type_): 当前车辆的位置x
            y (_type_): 当前车辆的位置y

        Returns:
            _type_: _description_
        """
        # 寻找参考轨迹最近目标点
        # d_x = [self.refer_path[i, 0]-x for i in range(len(self.refer_path))]
        # d_y = [self.refer_path[i, 1]-y for i in range(len(self.refer_path))]
        # d = [np.sqrt(d_x[i]**2+d_y[i]**2) for i in range(len(d_x))]
        d=np.linalg.norm(np.array([self.refer_path[:,0],self.refer_path[:,1]]).T-np.array([x,y]),axis=1,ord=2)
        s = np.argmin(d)  # 最近目标点索引
        d_x = self.refer_path[s, 0] - x
        d_y = self.refer_path[s, 1] - y
        yaw = self.refer_path[s, 2]
        k = self.refer_path[s, 3]
        yaw_now=np.arctan2(d_y,d_x)
        angle = normalize_angle(yaw - np.arctan2(d_y, d_x))
        e = d[s]  # 误差
        #print(f"距离为{e},下标为{s}")
        if angle < 0:
            e *= -1

        return e, k, yaw, s


class MyPath2:
    def __init__(self,dt=0.1,interval=[0,100],type=0):
        """使用了0-2*pi的角度表示方式,type=0为复杂轨迹，type=1为简单轨迹"""
        # set reference trajectory
        # refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k
        self.dt=dt#步长
        self.num=int((interval[1]-interval[0])/dt)#路径点个数
        self.interval=interval
        self.type=type
        self.refer_path = np.zeros((self.num, 4))
        self.refer_path[:, 0] = np.linspace(self.interval[0], self.interval[1], self.num)  # x
        if (self.type==0):
            self.refer_path[:, 1] = 2*np.sin(self.refer_path[:, 0]/3.0) + \
                2.5*np.cos(self.refer_path[:, 0]/2.0)  # y
        else:
            self.refer_path[:, 1] = (10*np.sin(self.refer_path[:, 0]*0.3))
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

    def calc_track_error(self, x, y):
        """计算跟踪误差

        Args:
            x (_type_): 当前车辆的位置x
            y (_type_): 当前车辆的位置y

        Returns:
            _type_: _description_
        """
        # 寻找参考轨迹最近目标点
        # d_x = [self.refer_path[i, 0]-x for i in range(len(self.refer_path))]
        # d_y = [self.refer_path[i, 1]-y for i in range(len(self.refer_path))]
        # d = [np.sqrt(d_x[i]**2+d_y[i]**2) for i in range(len(d_x))]
        d=np.linalg.norm(np.array([self.refer_path[:,0],self.refer_path[:,1]]).T-np.array([x,y]),axis=1,ord=2)
        s = np.argmin(d)  # 最近目标点索引
        d_x = self.refer_path[s, 0] - x
        d_y = self.refer_path[s, 1] - y
        yaw = self.refer_path[s, 2]
        k = self.refer_path[s, 3]
        yaw_now=np.arctan2(d_y,d_x)
        if (yaw_now<0):
            yaw_now+=2*np.pi
        angle = normalize_angle(yaw - yaw_now)
        e = d[s]  # 误差
        #print(f"距离为{e},下标为{s}")
        if angle < 0:
            e *= -1

        return e, k, yaw, s
    
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
            self.refer_path[:,1]=0.5*self.refer_path[:,1]
        else:
            self.refer_path[:,1]=0*self.refer_path[:,0]
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

    