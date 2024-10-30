#PID+MPC噪声分析
m_name = csvread("PID+MPC噪声分析平滑.csv",0,0,[0,0,0,2])
m1 = csvread("PID+MPC噪声分析平滑.csv",1,0,[1,0,106,0])
m2 = csvread("PID+MPC噪声分析平滑.csv",1,1,[1,1,114,1])
m3 = csvread("PID+MPC噪声分析平滑.csv",1,2,[1,2,114,2])

m1_x = range(1,length(m1))
m2_x = range(1,length(m2))
m3_x = range(1,length(m3))

figure()
plot(m1_x,m1,m2_x,m2,m3_x,m3,linewidth=2)
title("PID+MPC噪声分析")
legend(m_name)
xlabel("move(步长为0.125s)")
ylabel("误差")
grid("on")

##PID+PP算法噪声分析
p_name = csvread("PID+PP算法噪声分析平滑.csv",0,0,[0,0,0,2])
p1 = csvread("PID+PP算法噪声分析平滑.csv",1,0,[1,0,131,0])
p2 = csvread("PID+PP算法噪声分析平滑.csv",1,1,[1,1,133,1])
p3 = csvread("PID+PP算法噪声分析平滑.csv",1,2,[1,2,115,2])

p1_x = range(1,length(p1))
p2_x = range(1,length(p2))
p3_x = range(1,length(p3))

figure()
plot(p1_x,p1,p2_x,p2,p3_x,p3,linewidth=2)
title("PID+PP算法噪声分析")
legend(p_name)
xlabel("move(步长为0.125s)")
ylabel("误差")
grid("on")