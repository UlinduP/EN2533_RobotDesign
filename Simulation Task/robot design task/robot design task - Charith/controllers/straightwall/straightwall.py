from controller import Robot

timestep = 16
max_speed = 10
robot = Robot()

dis=[]
dis_Names=["dis_left","dis_front_left","dis_front_right","dis_right"]
dis_readings =[0]*(len(dis_Names)+2)
for name in dis_Names:
    dis.append(robot.getDevice(name))
    dis[-1].enable(timestep)

wheels = []
wheel_Names = ['right_front_wheel', 'left_front_wheel', 'right_rear_wheel', 'left_rear_wheel']
for name in wheel_Names:
    wheels.append(robot.getDevice(name))
    wheels[-1].setPosition(float('inf'))
    wheels[-1].setVelocity(0.0)

def getReadingDIS():
      
        if (int(dis[1].getValue())>int(dis[2].getValue()))and (int(dis[0].getValue())>400)  :
            dis_readings[1],dis_readings[2]=1,1
            dis_readings[3],dis_readings[4]=0,0
        elif int(dis[1].getValue())>900 and int(dis[2].getValue())> 900 and int(dis[0].getValue())>500 and int(dis[3].getValue())>500 :
            dis_readings[1],dis_readings[4]=0,0
            dis_readings[3],dis_readings[2]=1,1
        elif int(dis[2].getValue())>int(dis[1].getValue()) or (int(dis[0].getValue())<int(dis[3].getValue())): 
            dis_readings[1],dis_readings[2]=0,0
            dis_readings[3],dis_readings[4]=1,1
        
        for i in range(4):
             print(int(dis[i].getValue()), end=" ")
previous_error=0.0
kp=6#3
kd=0.5 #0.5
ki=0.0
Integral=0.0


def PID_wall():
    error=0
    coefficient=[-3000,-2000,-1000,1000,2000,3000]           
    for i in range(0,6):
              error+=coefficient[i]*dis_readings[i]
    P=kp*error
    I=Integral+(ki*error)
    D=kd*(error-previous_error)
    correction=(P+I+D)/1000
    l_speed=5+correction
    r_speed=5-correction
            
    if l_speed<0.0  : l_speed=0
    if l_speed>10.0 : l_speed=10.0
    if r_speed<0.0  : r_speed=0
    if r_speed>10.0 : r_speed=10.0
            
    wheels[0].setVelocity(r_speed)
    wheels[2].setVelocity(r_speed)
    wheels[1].setVelocity(l_speed)
    wheels[3].setVelocity(l_speed)
    print(l_speed,r_speed,dis_readings)           
    
    return I,error
while robot.step(timestep) != -1:
     getReadingDIS()

     print(kp, kd, ki)
    
    
     Integral,previous_error=PID_wall()

     pass
    
      
  