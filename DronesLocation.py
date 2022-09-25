#theta范围为【0，2pi】

import math
from pickletools import read_unicodestring1
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt

def arctan2(y,x):
    #math库中atan2函数输出范围为【-pi，pi】，转换为【0，2pi】
    return math.atan2(y,x)+math.pi

def distance(p1,p2):
    x1=p1[0]
    y1=p1[1]
    x2=p2[0]
    y2=p2[1]
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

def mirror(c1,c2,p):
    #p关于c1,c2连线的对称点
    #c1,c2连线方程为Ax+By+C=0
    x1=c1[0]
    y1=c1[1]
    x2=c2[0]
    y2=c2[1]
    x0=p[0]
    y0=p[1]
    A=y2-y1
    B=x1-x2
    C=(x2-x1)*y1-(y2-y1)*x1
    x=x0-2*A*(A*x0+B*y0+C)/(A**2+B**2)
    y=y0-2*B*(A*x0+B*y0+C)/(A**2+B**2)
    return[x,y]

def CountC(p1,p2,p3,a):
    #寻找圆轨迹的圆心
    x1=p1[0]
    y1=p1[1]
    x2=p2[0]
    y2=p2[1]
    if (y1-y2)*(x1-x2)>=0:
        if distance([(x1+x2)/2+abs(y1-y2)/2*math.tan(a),(y1+y2)/2-abs(x1-x2)/2*math.tan(a)],p3)<\
            distance([(x1+x2)/2-abs(y1-y2)/2*math.tan(a),(y1+y2)/2+abs(x1-x2)/2*math.tan(a)],p3):
            return [(x1+x2)/2+abs(y1-y2)/2*math.tan(a),(y1+y2)/2-abs(x1-x2)/2*math.tan(a)]
        else:
            return [(x1+x2)/2-abs(y1-y2)/2*math.tan(a),(y1+y2)/2+abs(x1-x2)/2*math.tan(a)]
    else:
        if distance([(x1+x2)/2+abs(y1-y2)/2*math.tan(a),(y1+y2)/2+abs(x1-x2)/2*math.tan(a)],p3)<\
            distance([(x1+x2)/2-abs(y1-y2)/2*math.tan(a),(y1+y2)/2-abs(x1-x2)/2*math.tan(a)],p3):
            return [(x1+x2)/2+abs(y1-y2)/2*math.tan(a),(y1+y2)/2+abs(x1-x2)/2*math.tan(a)]
        else:
            return [(x1+x2)/2-abs(y1-y2)/2*math.tan(a),(y1+y2)/2-abs(x1-x2)/2*math.tan(a)]   
        

def CountPoint(p1, p2, p3,a1,a2):
    #找出与点p1,p2,p3连线夹a1，a2角的点
    
    c1=CountC(p1,p2,p3,a1)
    c2=CountC(p1,p3,p2,a2)
    return mirror(c1,c2,p1)
    
def CountAngle(p1, p2, p3):
    #计算两点(p1,p2)分别与一点(p3)连线的夹角
    x1=p1[0]
    y1=p1[1]
    x2=p2[0]
    y2=p2[1]
    x3=p3[0]
    y3=p3[1]
    
    angle=sp.pi-sp.Abs(sp.pi-sp.Abs(sp.atan2(y1-y3,x1-x3)-sp.atan2(y2-y3,x2-x3)))    
    return angle


def draw(r,theta):
    #画出无人机分布散点图
    colors = theta

    ax = plt.subplot(111, projection='polar')
    c = ax.scatter(theta, r, c=colors, cmap='hsv', alpha=0.75)

    plt.show()
    
def ConvertXY(r,theta):
    #极坐标转换为xy坐标
    x=r*math.cos(theta)
    y=r*math.sin(theta)
    return [x,y]
    
def ConvertPolar(x,y):
    #xy坐标转换为极坐标
    r=math.sqrt(x**2+y**2)
    theta=arctan2(y,x)
    return[r,theta]
    
def loss(r,theta):
    #计算平均定位误差
    s=0
    for i in range(1,10,1):
        s=s+math.sqrt(r[i]**2+100**2-2*r[i]*100*math.cos(theta[i]-2*(i-1)*math.pi/9))
        
    l=s/9
    return l
        

def adjust(r,theta,targetloss,Ttheta1,Ttheta2):
    #r为0~9号无人机的极径   r[i]
    #theta为为0~9号无人机的极角   theta[i]
    #targetloss为目标误差
    #Ttheta1为以0，1号机为基准点i号无人机应接受到的目标夹角    Theta1[i]
    #Ttheta2为以0，n号机为基准点i号无人机应接受到的目标夹角    Theta2[n][i]
    
    draw(r,theta)
    
    #while loss(r,theta)>targetloss:
    for n in range(0,10,1):
            #0，1，n%8+2，(n+1)%8+2发射信号
            for i in range(2,10,1):
           #定位i号无人机
                if (i==n%8+2)|(i==(n+1)%8+2):
                    continue
                else:
                        point1=CountPoint(ConvertXY(r[0],theta[0]),ConvertXY(r[1],theta[1])\
                        ,ConvertXY(r[n%8+2],theta[n%8+2]),Ttheta1[i],Ttheta2[n%8+2][i]) 
                        print(point1)
                        point2=CountPoint(ConvertXY(r[0],theta[0]),ConvertXY(r[1],theta[1])\
                        ,ConvertXY(r[(n+1)%8+2],theta[(n+1)%8+2]),Ttheta1[i],Ttheta2[(n+1)%8+2][i])
                        #point1,point2分别为由0，1，n%8+2号机和0，1，(n+1)%7+2号机定位出i号机应在的点
                        
                        #x,y=sp.symbols('x,y',real=True)
                        #sp.solve([CountAngle(ConvertXY(r[0],theta[0]),ConvertXY(r[1],theta[1]),[x,y])\
                            #-Ttheta1[i],math.sqrt((point1[0]-x)**2+(point1[1]-y)**2)-\
                               # math.sqrt((point2[0]-x)**2+(point2[1]-y)**2)],[x,y])
                        
                        
                        x=(point2[0]+point1[0])/2
                        y=(point2[1]+point1[1])/2
                        point3=ConvertPolar(x,y)
                        r[i]=point3[0]
                        theta[i]=point3[1]
                        #最终定位为point1，point2所在圆弧中点
                
                
    draw(r,theta)
    print(r,theta)
    
def main():
    radius=[0,100,98,112,105,98,112,105,98,112]
    Theta=[0,0,40.1*math.pi/180,80.21*math.pi/180,119.75*math.pi/180,159.86*math.pi/180,199.96*math.pi/180,\
        240.07*math.pi/180,280.17*math.pi/180,320.28*math.pi/180]
    TargetTheta1=[0,0,7*math.pi/18,5*math.pi/18,math.pi/6,math.pi/18,math.pi/18,math.pi/6,\
        5*math.pi/18,7*math.pi/18]
    TargetTheta2=[[0,0,0,0,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0,0,0],
                  [0,0,0,7*math.pi/18,5*math.pi/18,math.pi/6,math.pi/18,math.pi/18,math.pi/6,5*math.pi/18],
                  [0,0,7*math.pi/18,0,7*math.pi/18,5*math.pi/18,math.pi/6,math.pi/18,math.pi/18,math.pi/6],
                  [0,0,5*math.pi/18,7*math.pi/18,0,7*math.pi/18,5*math.pi/18,math.pi/6,math.pi/18,math.pi/18],
                  [0,0,math.pi/6,5*math.pi/18,7*math.pi/18,0,7*math.pi/18,5*math.pi/18,math.pi/6,math.pi/18],
                  [0,0,math.pi/18,math.pi/6,5*math.pi/18,7*math.pi/18,0,7*math.pi/18,5*math.pi/18,math.pi/6],
                  [0,0,math.pi/18,math.pi/18,math.pi/6,5*math.pi/18,7*math.pi/18,0,7*math.pi/18,5*math.pi/18],
                  [0,0,math.pi/6,math.pi/18,math.pi/18,math.pi/6,5*math.pi/18,7*math.pi/18,0,7*math.pi/18],
                  [0,0,5*math.pi/18,math.pi/6,math.pi/18,math.pi/18,math.pi/6,5*math.pi/18,7*math.pi/18,0]]
    
    adjust(radius,Theta,0.00001,TargetTheta1,TargetTheta2)
    
if __name__== "__main__" :
    main()
        
        
        
        
    