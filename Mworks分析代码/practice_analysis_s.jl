#调整kp
p_name = csvread("实际误差平滑.csv",0,1,[0,1,0,3])
p1 = csvread("实际误差平滑.csv",1,1,[1,1,500,1])
p2 = csvread("实际误差平滑.csv",1,2,[1,2,500,2])
p3 = csvread("实际误差平滑.csv",1,3,[1,3,500,3])

p1_x = range(1,length(p1))
p2_x = range(1,length(p2))
p3_x = range(1,length(p3))

figure()
plot(p1_x,p1,p2_x,p2,p3_x,p3,linewidth=2)
title("实机-调整kp")
legend(p_name)
xlabel("move(步长为0.05s)")
ylabel("误差")
grid("on")

#调整kd
p_name = csvread("实际误差平滑.csv",0,5,[0,5,0,7])
p1 = csvread("实际误差平滑.csv",1,5,[1,5,500,5])
p2 = csvread("实际误差平滑.csv",1,6,[1,6,500,6])
p3 = csvread("实际误差平滑.csv",1,7,[1,7,500,7])

p1_x = range(1,length(p1))
p2_x = range(1,length(p2))
p3_x = range(1,length(p3))

figure()
plot(p1_x,p1,p2_x,p2,p3_x,p3,linewidth=2)
title("实机-调整kd")
legend(p_name)
xlabel("move(步长为0.05s)")
ylabel("误差")
grid("on")

#调整ki
p_name = csvread("实际误差平滑.csv",0,9,[0,9,0,12])
p1 = csvread("实际误差平滑.csv",1,9,[1,9,500,9])
p2 = csvread("实际误差平滑.csv",1,10,[1,10,500,10])
p3 = csvread("实际误差平滑.csv",1,11,[1,11,500,11])
p4 = csvread("实际误差平滑.csv",1,12,[1,12,500,12])

p1_x = range(1,length(p1))
p2_x = range(1,length(p2))
p3_x = range(1,length(p3))
p4_x = range(1,length(p4))

figure()
plot(p1_x,p1,p2_x,p2,p3_x,p3,p4_x,p4,linewidth=2)
title("实机-调整ki")
legend(p_name)
xlabel("move(步长为0.05s)")
ylabel("误差")
grid("on")