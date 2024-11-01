#调整kp
p_name = csvread("PID+PP算法参数分析平滑.csv",0,1,[0,1,0,3])
p1 = csvread("PID+PP算法参数分析平滑.csv",1,1,[1,1,138,1])
p2 = csvread("PID+PP算法参数分析平滑.csv",1,2,[1,2,133,2])
p3 = csvread("PID+PP算法参数分析平滑.csv",1,3,[1,3,132,3])

p1_x = range(1,length(p1))
p2_x = range(1,length(p2))
p3_x = range(1,length(p3))
figure()
plot(p1_x,p1,p2_x,p2,p3_x,p3,linewidth=2)
title("PID+PP算法参数分析-调整kp")
legend(p_name)
xlabel("move(步长为0.125s)")
ylabel("误差")
grid("on")

#调整kd
p_name = csvread("PID+PP算法参数分析平滑.csv",0,5,[0,5,0,8])
p1 = csvread("PID+PP算法参数分析平滑.csv",1,5,[1,5,133,5])
p2 = csvread("PID+PP算法参数分析平滑.csv",1,6,[1,6,131,6])
p3 = csvread("PID+PP算法参数分析平滑.csv",1,7,[1,7,131,7])
p4 = csvread("PID+PP算法参数分析平滑.csv",1,8,[1,8,131,8])

p1_x = range(1,length(p1))
p2_x = range(1,length(p2))
p3_x = range(1,length(p3))
p4_x = range(1,length(p4))

figure()
plot(p1_x,p1,p2_x,p2,p3_x,p3,p4_x,p4,linewidth=2)
title("PID+PP算法参数分析-调整kd")
legend(p_name)
xlabel("move(步长为0.125s)")
ylabel("误差")
grid("on")

#调整ki
p_name = csvread("PID+PP算法参数分析平滑.csv",0,10,[0,10,0,12])
p1 = csvread("PID+PP算法参数分析平滑.csv",1,10,[1,10,131,10])
p2 = csvread("PID+PP算法参数分析平滑.csv",1,11,[1,11,131,11])
p3 = csvread("PID+PP算法参数分析平滑.csv",1,12,[1,12,131,12])
p4 = csvread("PID+PP算法参数分析平滑.csv",1,13,[1,13,131,13])

p1_x = range(1,length(p1))
p2_x = range(1,length(p2))
p3_x = range(1,length(p3))
p4_x = range(1,length(p4))

figure()
plot(p1_x,p1,p2_x,p2,p3_x,p3,p4_x,p4,linewidth=2)
title("PID+PP算法参数分析-调整ki")
legend(p_name)
xlabel("move(步长为0.125s)")
ylabel("误差")
grid("on")