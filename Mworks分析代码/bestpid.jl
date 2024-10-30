function maxl(l)
    max = -9999
    for i in l
        if i > max
            max = i
        end
    end
    return max
end

#实机best
#p1 = csvread("实机误差.csv",1,1,[1,1,500,1])
p1 = csvread("实际误差平滑.csv",1,11,[1,11,500,11])
maxp1 = maxl(p1)
p1_x = range(1,length(p1))

figure()
#ssv = -0.002
ssv = 0.007
print(maxp1,maxp1-ssv,"\n")
ssv_r = 0.05*(maxp1-ssv)
s = [ssv for i = range(1,length(p1_x))]
s_u = [ssv+ssv_r for i = range(1,length(p1_x))]
s_d = [ssv-ssv_r for i = range(1,length(p1_x))]
plot(p1_x,p1,p1_x,s,"--",p1_x,s_u,"--",p1_x,s_d,"--",linewidth=2)
title("实机-best")
legend(["Kp=3,Ki=0.007,Kd=0","稳态值","5%误差范围","5%误差范围"])
xlabel("move(步长为0.05s)")
ylabel("误差")
grid("on")

#测试best
#p2 = csvread("PID+PP算法参数分析.csv",1,15,[1,15,127,15])
p2 = csvread("PID+PP算法参数分析平滑.csv",1,11,[1,11,131,11])
maxp2 = maxl(p2)
p2_x = range(1,length(p2))

figure()
ssv = -0.007
print(maxp2,maxp2-ssv,"\n")
ssv_r = 0.05*(maxp1-ssv)
s = [ssv for i = range(1,length(p2_x))]
s_u = [ssv+ssv_r for i = range(1,length(p2_x))]
s_d = [ssv-ssv_r for i = range(1,length(p2_x))]
plot(p2_x,p2,p2_x,s,"--",p2_x,s_u,"--",p2_x,s_d,"--",linewidth=2)
title("测试-best")
legend(["kp=3,ki=0.001,kd=6","稳态值","5%误差范围","5%误差范围"])
xlabel("move(步长为0.125s)")
ylabel("误差")
grid("on")