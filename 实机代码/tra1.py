"""PID结合PP算法控制小车轨迹"""
import robomaster
from robomaster import robot
from robomaster import led
from robomaster import blaster
from robomaster import camera
import time
import numpy as np
import copy
from Path import CarPath
from PID import PID
import matplotlib.pyplot as plt
import pandas as pd

plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号
class Car:
    def __init__(self):
        self.car=robot.Robot()
        try:
            self.car.initialize(conn_type='ap')
            print("机器人已连接")
        except:
            raise Exception("无法连接到机器人")
        self.chassis=self.car.chassis
        self.battery=self.car.battery
        self.gimbal=self.car.gimbal
        self.gimbal.recenter().wait_for_completed()
        self.length=0.25
        self.v=0.1
        self.__init_pos=()#x,y,x上电时刻
        self.__cur_pos=()#x,y,z当前时刻
        self.__vel=()#vgx,vgy,vgz,vbx,vby,vbz(上电时刻，当前时刻)
        self.__impact=()#impactx,impacty,impactz
        self.__battery=0#电量
        self.__yaw=0#航向角


    def __query_cur_pos(self,res):
        self.__cur_pos=res
    def __query_vel(self,res):
        self.__vel=res
    def __query_status(self,res):
        self.__impact=(res[6],res[7],res[8])
    def __query_battery(self,percent):
        self.__battery=percent
    def __query_yaw(self,res):
        self.__yaw=res[0]


    def sub_infomation(self):
        self.chassis.sub_position(0,freq=20,callback=self.__query_cur_pos)
        self.chassis.sub_attitude(freq=20,callback=self.__query_yaw)
        self.chassis.sub_velocity(freq=20,callback=self.__query_vel)
        self.chassis.sub_status(freq=20,callback=self.__query_status)
        self.battery.sub_battery_info(freq=1,callback=self.__query_battery)


    def unsub_information(self):
        self.chassis.unsub_position()
        self.chassis.unsub_velocity()
        self.chassis.unsub_status()
        self.battery.unsub_battery_info()

    def get_cur_pos(self):
        return self.__cur_pos
    def get_velocity(self):
        return self.__vel
    def get_impact(self):
        return self.__impact
    def get_battery(self):
        return self.__battery
    def get_yaw(self):
        return self.__yaw
    

    def move_pos(self, steering, distance, max_steering_angle=np.pi / 8.0):
        """
        按位置移动
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0
        steering=steering/np.pi*180
        carmove=self.chassis.move(x=distance,y=0,z=-steering)
        flag=True
        if any(self.__impact):
            print("检测到碰撞")
            carmove=self.chassis.move(x=0,y=0,z=0)#停止
            flag=False
        carmove.wait_for_completed()
        return flag

    def move_vel(self, steering, max_steering_angle=np.pi / 8.0):
        """
        按速度移动
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        steering=steering/np.pi*180
        self.chassis.drive_speed(x=self.v,y=0,z=steering)
        flag=True
        if any(self.__impact):
            print("检测到碰撞")
            self.chassis.drive_speed(x=0,y=0,z=0)#停止
            flag=False
        return flag
    
    def close_robot(self):
        self.car.close()
        print("已关闭")
        self.car=None


def show_battery(percent):
    print(f"电量是{percent}")

def show_position(res):
    print(f"x位置：{res[0]}")
    #print("what")

def show_mode(mode):
    print(mode)

def show_v(res):
    print(f"x速度{res[3]}")

def rad2angle(x):
    return x/np.pi*180

def angle2rad(x):
    return x/180*np.pi

def PID_track(mycar,path):
    #pid=PID(1,5,0.001,8)
    pid=PID(3,0.007,0)
    num=path.num
    tra=[]
    y_err=[]
    l_d=0.3#前视距离
    for i in range(500):
        cur_pos=mycar.get_cur_pos()
        cur_pos=np.array([cur_pos[0],cur_pos[1]])
        tra.append([cur_pos[0],cur_pos[1]])
        cur_yaw=mycar.get_yaw()
        print(f"当前位置和角度为，x:{cur_pos[0]},y:{cur_pos[1]},z:{cur_yaw}")    
        dis=np.linalg.norm(cur_pos-np.array([path.refer_path[:,0],path.refer_path[:,1]]).T,axis=1,ord=2)
        dis_min=np.min(dis)
        min_id=np.argmin(dis)
        if np.abs(min_id-num)<=3:
            break

        """预瞄算法"""
        new_min_id=min_id
        new_dis=dis_min

        while new_min_id<num-2 and new_dis<l_d:
            #前视预瞄
            new_min_id+=1
            new_dis=dis[new_min_id]

        new_theta_e=np.arctan2(path.refer_path[new_min_id][1]-cur_pos[1],path.refer_path[new_min_id][0]-cur_pos[0])-angle2rad(cur_yaw)
        print(f"预期位置和角度为，x:{path.refer_path[min_id][0]},y:{path.refer_path[min_id][1]},z:{rad2angle(path.refer_path[min_id][2])}")
        dy=path.refer_path[min_id][1]-cur_pos[1]
        dx=path.refer_path[min_id][0]-cur_pos[0]
        theta_e=(np.arctan2(dy,dx)-angle2rad(cur_yaw))%(2*np.pi)#角度误差
        e_y=dis_min*np.sin(theta_e)#横向误差
        y_err.append(e_y)
        e_d=0.4
        new_e_y=new_dis*np.sin(new_theta_e)
        pid.update_e(new_e_y)

        #u_w=np.arctan(2*mycar.length*np.sin(new_theta_e)/l_d)
        u_w=pid.get_u()
        if not mycar.move_vel(u_w,e_d):
            break
        print(f"第{i}次，转角位{rad2angle(u_w)}")
        try:
            time.sleep(0.05)
        except:
            mycar.close_robot()
    return tra,y_err

if __name__ == '__main__':
    mycar=Car()
    mycar.sub_infomation()
    time.sleep(2)
    path=CarPath(dt=0.01,interval=[0,3],type=0)
    tra,y_err=PID_track(mycar,path)
    mycar.close_robot()
    # df=pd.DataFrame(y_err,columns=["error,Kp=3,Ki=0.007,Kd=0"])
    # df.to_csv("error1.csv")
    x_tra=[x[0] for x in tra]
    y_tra=[x[1] for x in tra]
    time.sleep(2)
    fig,ax=plt.subplots(2)
    fig.suptitle("Kp=3,Ki=0.007,Kd=0的实机结果")
    ax[0].plot(path.refer_path[:,0],path.refer_path[:,1])
    ax[0].plot(x_tra,y_tra)
    ax[0].set_xlabel('x')
    ax[0].set_ylabel('y')
    ax[0].set_title("小车轨迹")
    ax[1].grid(True)
    ax[1].plot(y_err)
    ax[1].set_xlabel("步数")
    ax[1].set_ylabel("横向误差")
    ax[1].set_title("小车误差")
    plt.show()
    # print(mycar.get_cur_pos())

    # mycar.chassis.move(x=0.5,y=0,z=0,xy_speed=0.7).wait_for_completed()
    # print(mycar.get_cur_pos())
    # print(mycar.get_velocity())
    # mycar.chassis.move(x=-0.5,y=0,z=0,xy_speed=0.7).wait_for_completed()
    # print(mycar.get_cur_pos())
    # print(mycar.get_velocity())
    # time.sleep(2)
    # mycar.unsub_information()
    # mycar.close_robot()
