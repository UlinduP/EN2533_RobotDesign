from controller import Robot

timestep = 16
max_speed = 5
robot = Robot()

ir = []
ir_Names = ["ir_ext_left","ir_left","ir_middle_left","ir_middle_right","ir_right","ir_ext_right"]
ir_readings = [0]*len(ir_Names)

dis=[]
dis_Names=["dis_front","dis_right","dis_left"]
dis_val =[0]*len(dis_Names)

for name in ir_Names:
    ir.append(robot.getDevice(name))
    ir[-1].enable(timestep)

for name in dis_Names:
    dis.append(robot.getDevice(name))
    dis[-1].enable(timestep)

wheels = []
wheel_Names = ['right_front_wheel', 'left_front_wheel', 'right_rear_wheel', 'left_rear_wheel']
for name in wheel_Names:
    wheels.append(robot.getDevice(name))
    wheels[-1].setPosition(float('inf'))
    wheels[-1].setVelocity(0.0)


def getReadingIR():

    for i in range (0,6):
        if int(ir[i].getValue())>512:
            ir_readings[i]=0
        elif int(ir[i].getValue())< 180:
            ir_readings[i]=1

previous_error=0.0
kp=6 #3
kd=0.0 #0.5
ki=0.0
Integral=0.0

def PID_line():

    error=0
    coefficient=[-3000,-2000,-1000,1000,2000,3000]           
  
    for i in range(0,6):
              error+=coefficient[i]*ir_readings[i]
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
    
    print(l_speed,r_speed,ir_readings)
    
    return I,error
while (robot.step(timestep) != -1):

    getReadingIR()

    print(kp, kd, ki)
    if ir_readings[0]==1 and ir_readings[1]==1 and ir_readings[2]==1 and ir_readings[3]==1 and ir_readings[4]==0 and ir_readings[5]==0 :
         print("***************")
       
         while (ir_readings[2]==1 and ir_readings[3]==1 and ir_readings[0]==0)!="True":
                r_speed=10
                l_speed=-10           
                wheels[0].setVelocity(r_speed)
                wheels[2].setVelocity(r_speed)
                wheels[1].setVelocity(l_speed)
                wheels[3].setVelocity(l_speed)
                print(l_speed,r_speed,ir_readings,"******")
                getReadingIR()
    else:
          Integral,previous_error=PID_line()

    pass