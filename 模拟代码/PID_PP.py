"""神经网络自适应PID控制器,但是没实现，改成了纯跟踪算法,老师又说要用PID，于是把PP跟PID结合了一下"""
import numpy as np
import matplotlib.pyplot as plt
from PIDRobotClass import Robot
from PID import PID_inc,PID#增量式PID
from ReferencePath import MyPath2
import pandas as pd

plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号
# time=100
# dt=4*np.pi/100
# path=MyPath2(dt,[-2*np.pi,2*np.pi],1)
time=1000
dt=0.1
path=MyPath2(dt,[0,100],0)

wheellengh = 1
rob = Robot(wheellengh); 
#rob.set_noise(0.1, 0.1)
#rob.set_steering_drift(0.1)
start = [path.refer_path[0,0]+3,path.refer_path[0,1]+3,path.refer_path[0,2]]
rob.set(*start)



def model_PID(kp,ki,kd):
    #pid=PID(2,0.02,6)
    #pid=PID(3,0,6)
    #pid_inc=PID_inc([1.5,0.1,10]
    rob.set(*start)
    y_err=[]
    pid=PID(kp,ki,kd)
    tra=[start]
    l_d=0.5#前视距离
    for i in range(1000):
        dis=np.linalg.norm(np.array([rob.x,rob.y])-np.array([path.refer_path[:,0],path.refer_path[:,1]]).T,axis=1,ord=2)
        dis_min=np.min(dis)
        min_id=np.argmin(dis)
        new_min_id=min_id
        new_dis=dis_min

        while new_min_id<time-2 and new_dis<l_d:
            #前视预瞄
            new_min_id+=1
            new_dis=dis[new_min_id]

        theta_e=np.arctan2(path.refer_path[min_id][1]-rob.y,path.refer_path[min_id][0]-rob.x)-rob.orientation#角度误差
        if np.abs(min_id-time)<=4:
            break

        e_y=dis_min*np.sin(theta_e)#横向误差

        new_theta_e=np.arctan2(path.refer_path[new_min_id][1]-rob.y,path.refer_path[new_min_id][0]-rob.x)-rob.orientation
        #u_w=np.arctan(2*rob.length*np.sin(new_theta_e)/l_d)
        y_err.append(e_y)
        e_d=0.2
        new_e_y=new_dis*np.sin(new_theta_e)
        pid.update_e(new_e_y)
        u_w=pid.get_u()
        #u_w=pid_inc.cal_output(e_y)
        #u_w=pid.result
        #print(pid.wp,pid.wi,pid.wd,pid.result)
        #print(u_w)
        rob.move(u_w,e_d)
        tra.append([rob.x,rob.y,rob.orientation])
    x_tra=[x[0] for x in tra]
    y_tra=[x[1] for x in tra]
    return x_tra,y_tra,y_err


def get_error_data():  
    data={}
    for i in range(1,6):
        x_tra,y_tra,y_err=model_PID(i,0,0)
        data["kp="+str(i)+"ki=0kd=0"]=y_err
    for i in range(0,9,2):
        x_tra,y_tra,y_err=model_PID(3,0,i)
        data["kp=3ki=0kd="+str(i)]=y_err
    x_tra,y_tra,y_err=model_PID(3,0.001,6)
    data["kp=3ki=0.001kd=6"]=y_err
    x_tra,y_tra,y_err=model_PID(3,0.01,6)
    data["kp=3ki=0.01kd=6"]=y_err
    x_tra,y_tra,y_err=model_PID(3,0.1,6)
    data["kp=3ki=0.1kd=6"]=y_err
    max_length = max([len(data[key]) for key in data])
    for key in data:
        current_length = len(data[key])
        print(current_length)
        if current_length < max_length:
            data[key] = np.append(data[key], [np.NaN] * (max_length - current_length))
    df=pd.DataFrame(data)
    df.to_csv("error1.csv",index=False)

def get_noise_data():
    data={}
    drift=0.1
    noise=0
    step=0.05
    for i in range(1,6):
        rob.set_noise(noise,noise)
        rob.set_steering_drift(drift)
        x_tra,y_tra,y_err=model_PID(3,0.001,6)
        data["noise="+str(noise)+"drift="+str(drift)]=y_err
        noise+=step    
    max_length = max([len(data[key]) for key in data])
    for key in data:
        current_length = len(data[key])
        print(current_length)
        if current_length < max_length:
            data[key] = np.append(data[key], [np.NaN] * (max_length - current_length))
    df=pd.DataFrame(data)
    df.to_csv("error1.csv",index=False)

# rob.set_noise(0.1, 0.1)
# rob.set_steering_drift(0.1)
x_tra,y_tra,y_err=model_PID(3,0.001,6)
fig=plt.figure()
fig.suptitle("PP+PID结果")
plt.plot(path.refer_path[:,0],path.refer_path[:,1])
plt.plot(x_tra,y_tra)
plt.xlabel('x')
plt.ylabel('y')
plt.title("小车轨迹")
plt.show()

# fig,ax=plt.subplots(2)
# fig.suptitle("PP+PID结果")
# ax[0].plot(path.refer_path[:,0],path.refer_path[:,1])
# ax[0].plot(x_tra,y_tra)
# ax[0].set_xlabel('x')
# ax[0].set_ylabel('y')
# ax[0].set_title("小车轨迹")
# ax[1].grid(True)
# ax[1].plot(y_err)
# ax[1].set_xlabel("步数")
# ax[1].set_ylabel("横向误差")
# ax[1].set_title("小车误差")
plt.show()

