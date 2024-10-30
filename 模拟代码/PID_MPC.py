"""解决了MPC角度周期突变问题"""

import numpy as np
import matplotlib.pyplot as plt
from PIDRobotClass import Robot
from PID import PID
from ReferencePath import MyPath2
import cvxpy
import copy
import random
import pandas as pd
plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号
#生成路径
time=1000
dt=0.1
ref_path=MyPath2(0.1,[0,100],0)
# time=100
# dt=4*np.pi/100
# ref_path=MyPath2(dt,[-2*np.pi,2*np.pi],1)
#设置机器人参数
wheellengh = 1
rob = Robot(wheellengh); 
# rob.set_noise(0.2,0.2)
# rob.set_steering_drift(0.1)
start = [ref_path.refer_path[0,0]+3,ref_path.refer_path[0,1]+3,ref_path.refer_path[0,2]]
rob.set(*start)

#设置MPC参数
NX = 3  # 状态量[x,y,yaw]
NU = 2  # 控制量[v,delta]
T=20#预测步数

R = np.diag([0.1, 0.1])  # 输入代价矩阵
Rd = np.diag([0.1, 0.1])  # 输入差分代价矩阵
Q = np.diag([1, 1, 1])  # 状态代价矩阵
Qf = Q  # 最终状态代价矩阵
MAX_VEL = 10  # 最大速度
MAX_STEER = np.pi/4  # 最大转角
#记录误差
theta_err=[]



def get_nparray_from_matrix(x):
    return np.array(x).flatten()
def trans_state_space(ref_delta, ref_yaw,ref_v,length,dt):
    """将模型离散化后的状态空间表达

    Args:
        ref_delta (_type_): 参考的转角控制量
        ref_yaw (_type_): 参考的偏航角
        dt: 时间间隔
        ref_v: 参考速度
        length: 车辆的轴距
    Returns:
        A,B,C: 相应的系数矩阵
    """

    A = np.matrix([
        [1.0, 0.0, -ref_v*dt*np.sin(ref_yaw)],
        [0.0, 1.0, ref_v*dt*np.cos(ref_yaw)],
        [0.0, 0.0, 1.0]])

    B = np.matrix([
        [dt*np.cos(ref_yaw), 0],
        [dt*np.sin(ref_yaw), 0],
        [dt*np.tan(ref_delta)/length, ref_v*dt /(length*np.cos(ref_delta)*np.cos(ref_delta))]
    ])

    C = np.eye(3)
    return A, B, C
def linear_mpc_control(xref, x0, delta_ref, rob,pre_v,steer_noise=0.0,v_noise=0.0):
    """
    linear mpc control

    xref: reference point
    x0: initial state
    delta_ref: reference steer angle
    rob:车辆对象
    pre_v:车辆上次速度
    returns: 最优的控制量和最优状态
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0  # 代价函数
    constraints = []  # 约束条件
    # v_noise=random.gauss(delta_ref[0,:],v_noise)-delta_ref[0,:]
    # steer_noise=random.gauss(delta_ref[1,:],steer_noise)-delta_ref[1,:]
    # noise=np.array([v_noise,steer_noise]).reshape((NU,T))
    for t in range(T):
        cost += cvxpy.quad_form(u[:, t]-delta_ref[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(x[:, t] - xref[:, t], Q)

        A, B, C = trans_state_space(delta_ref[1, t], xref[2, t],pre_v,rob.length,dt)#v为参考速度
        constraints += [x[:, t + 1]-xref[:, t+1] == A @
                        (x[:, t]-xref[:, t]) + B @ (u[:, t]-delta_ref[:, t])]


    cost += cvxpy.quad_form(x[:, T] - xref[:, T], Qf)

    constraints += [(x[:, 0]) == x0]
    #constraints+=[MIN_VEL<=cvxpy.abs(u[0,:])]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_VEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.CLARABEL, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        opt_x = get_nparray_from_matrix(x.value[0, :])
        opt_y = get_nparray_from_matrix(x.value[1, :])
        opt_yaw = get_nparray_from_matrix(x.value[2, :])
        opt_v = get_nparray_from_matrix(u.value[0, :])
        opt_delta = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        opt_v, opt_delta, opt_x, opt_y, opt_yaw = None, None, None, None, None,

    return opt_v, opt_delta, opt_x, opt_y, opt_yaw
def calc_ref_tra(rob,v,ref_path,ind,dl=1.0):
    """计算参考轨迹点，统一化变量数组，便于后面MPC优化使用,
    其中rob为车辆，v为上次速度，ind为目标轨迹上离车辆最近的点，ref_path为参考路径"""
    robot_state=np.zeros(4)
    robot_state[0]=rob.x
    robot_state[1]=rob.y
    robot_state[2]=rob.orientation
    robot_state[3]=v
    k=ref_path.refer_path[ind,3]#曲率，表示角度预期的变化
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((NU, T))
    ncourse = len(ref_path.refer_path)
    xref[0, 0] = ref_path.refer_path[ind, 0]
    xref[1, 0] = ref_path.refer_path[ind, 1]
    xref[2, 0] = ref_path.refer_path[ind, 2]
    # 参考控制量[v,delta]
    ref_delta = (np.arctan2(rob.length*k, 1))#参考变化角度，根据曲率计算
    #print(f"参考delta角为{ref_delta}")
    dref[0, :] = robot_state[3]
    dref[1, :] = ref_delta
    travel = 0.0
    for i in range(T + 1):
        travel += abs(robot_state[3]) * dt
        dind = int(round(travel / dl))
        if (ind + dind) < ncourse:
            xref[0, i] = ref_path.refer_path[ind + dind, 0]
            xref[1, i] = ref_path.refer_path[ind + dind, 1]
            xref[2, i] = ref_path.refer_path[ind + dind, 2]
        else:
            xref[0, i] = ref_path.refer_path[ncourse - 1, 0]
            xref[1, i] = ref_path.refer_path[ncourse - 1, 1]
            xref[2, i] = ref_path.refer_path[ncourse - 1, 2]
    return xref, dref,ref_delta   
def model_PID_MPC():
    #pid=PID(2,0.01,28)
    y_err=[]
    #pid=PID(2,0.01,40)
    #pid=PID(2,0.001,40)
    pid=PID(3,0.001,6)
    tra=[start]
    pre_v=2 #上次速度,单位m/s
    x0=copy.deepcopy(start)
    pre_opt_delta=0
    last_ey=1e9
    ld=0.3#前视距离
    for i in range(1000):
        dis=np.linalg.norm(np.array([rob.x,rob.y])-np.array([ref_path.refer_path[:,0],ref_path.refer_path[:,1]]).T,axis=1,ord=2)
        dis_min=np.min(dis)
        min_id=np.argmin(dis)
        if (time-min_id)<=3:
            break
        new_min_id=min_id
        new_dis=dis_min

        while new_min_id<time-2 and new_dis<ld:
            #前视预瞄
            new_min_id+=1
            new_dis=dis[new_min_id]

        new_theta_e=np.arctan2(ref_path.refer_path[new_min_id][1]-rob.y,ref_path.refer_path[new_min_id][0]-rob.x)-rob.orientation
        new_e_y=new_dis*np.sin(new_theta_e)

        dy=ref_path.refer_path[min_id][1]-rob.y
        dx=ref_path.refer_path[min_id][0]-rob.x
        #print(rob.orientation)
        xref, dref ,ref_delta= calc_ref_tra(rob,pre_v,ref_path,min_id)#计算参考轨迹以及参考控制量,k为预期角度变化
        opt_v, opt_delta, opt_x, opt_y, opt_yaw=linear_mpc_control(xref, x0, dref, rob,pre_v)
        theta_e=(np.arctan2(dy,dx)-rob.orientation)%(2*np.pi)#角度误差
        e_y=dis_min*np.sin(theta_e)#横向误差

        last_ey=e_y
        #print(f"角度误差为{theta_e},最优偏移角为{opt_delta[0]}")
        y_err.append(e_y)
        e_d=np.abs(opt_v[0]*dt)
        e_d=max(0.1,e_d)
        pid.update_e(new_e_y)
        u_w=pid.get_u()
        #if np.abs(opt_delta[0]-pre_opt_delta)<=(np.pi/4) or np.abs(e_y)>0.2:
        print(i)
        print(opt_delta[0])
        #if rob.orientation>np.pi/4 and rob.orientation<2*np.pi-np.pi/4 and np.abs(opt_delta[0])<np.pi/5:
        if i<100 or (np.pi/4-np.abs(opt_delta[0]))<=0.0001:
            rob.move(u_w,0.2)
            print("pid")
        else:
            rob.move(opt_delta[0],e_d)
            #pre_opt_delta=opt_delta[0]
            #print(e_d)
            print("mpc")
        #rob.move(opt_delta[0],e_d)
        #print(f"速度为{opt_v[0]*dt},角度为{opt_delta[0]}")
        #pre_v=opt_v[0]
        x0=np.array([rob.x,rob.y,rob.orientation])
        tra.append([rob.x,rob.y,rob.orientation])
    x_tra=[x[0] for x in tra]
    y_tra=[x[1] for x in tra]
    return x_tra,y_tra,y_err


x_tra,y_tra,y_err=model_PID_MPC()
df=pd.DataFrame({"noise=0.05,drift=0.1":y_err})
df.to_csv("error1.csv")
fig,ax=plt.subplots(2)
fig.suptitle("PID_MPC结果")
ax[0].plot(ref_path.refer_path[:,0],ref_path.refer_path[:,1])
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

