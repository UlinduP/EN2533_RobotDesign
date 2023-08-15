from controller import Robot

TIME_STEP = 16
max_speed = 5
robot = Robot()
ds = []
ds_Names = ["ir_middle_right","ir_right","ir_left","ir_ext_right","ir_ext_left","dis_front","dis_right","dis_left","ir_middle_left"]
ds_val = [0]*len(ds_Names)

for name in ds_Names:
    ds.append(robot.getDevice(name))
    ds[-1].enable(TIME_STEP)
wheels = []
wheel_Names = ['right_front_wheel', 'left_front_wheel', 'right_rear_wheel', 'left_rear_wheel']
for name in wheel_Names:
    wheels.append(robot.getDevice(name))
    wheels[-1].setPosition(float('inf'))
    wheels[-1].setVelocity(0.0)

last_error=intg=diff=prop=0
kp=0.0001
ki=0
kd=0.005

def pid(error):
    global last_error,intg,diff,prop,kp,ki,kd
    prop = error
    intg = error + intg
    diff = error - last_error
    balance = kp*prop + kd*diff + ki*intg
    last_error= error
    return balance

def setSpeed(base_speed,balance):
     wheels[0].setVelocity(base_speed-balance)
     wheels[1].setVelocity(base_speed+balance)
     wheels[2].setVelocity(base_speed-balance)
     wheels[3].setVelocity(base_speed+balance)    

while robot.step(TIME_STEP) != -1:
    
    for i in range(len(ds)):
         ds_val[i] = ds[i].getValue()
         print(f"{ds_Names[i]} : {ds_val[i]}\n" + "*" *40)
    """if ds_val[0]< 500 and ds_val[1]< 500 and ds_val[2]<500 and ds_val[3] < 500 and ds_val[4] <500:
         setSpeed(5,0)
    if  ds_val[1] >340 and ds_val[2] <170 and ds_val[3] >340 and ds_val[4] <170:
         setSpeed(2.5,-5)
    if ds_val[0]"""
   
    """ if ds_val[5]== 1000 and ds_val[7]==1000 and ds_val[6]<1000:  
        error = ds_val[6] - 300
        rectify = pid(error)
        setSpeed(5,rectify)
    if ds_val[-3]!=1000 and ds_val[-2]<1000 and ds_val[-1] == 1000:
        setSpeed(2.5,-5)"""
         
      