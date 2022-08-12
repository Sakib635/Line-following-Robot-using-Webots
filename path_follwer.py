"""path_follwer controller."""


from controller import Robot

#create the Robot Instance.

robot = Robot()

#get the time step of the current world. 
timestep = int(robot.getBasicTimeStep())

time_step = 64

speed = 7


#motor

wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

#wheel2 wheel4 right side index 1,3
#wheel1 wheel3 left side index 0,2
#Ir Sensor

right_ir = robot.getDevice('RIGHT S')

right_ir.enable(time_step)

mid_ir = robot.getDevice('MID S')

mid_ir.enable(time_step)

left_ir = robot.getDevice('LEFT S')

left_ir.enable(time_step)




#PID
last_error = intg = diff = prop = weight_counter = 0
kp = 0.03
ki = 0.0001
kd = 0.5

def pid(error):
    global last_error, intg, diff, prop, kp, ki, kd
    prop = error
    intg = error + intg
    diff = error - last_error
    balance = (kp * prop) + (ki * intg) + (kd * diff)
    last_error = error 
    return balance
#if balance is negetive robot will move left, and if balance is
#positive robot will move to right
def set_speed (speed, balance):
    wheels[0].setVelocity(speed + balance)
    wheels[1].setVelocity(speed - balance)
    
    wheels[2].setVelocity(speed + balance)
    wheels[3].setVelocity(speed - balance)


#perform simulation steps until webots is stopping the controller
while robot.step(timestep) != -1:

    #Read the sensors:
    
    #Enter here functions to read sensor date, like:
    
    #val ds.getValue()
    
    right_ir_val = right_ir.getValue()
    mid_ir_val = mid_ir.getValue()
    left_ir_val = left_ir.getValue()
    
    
    
    #print("left: {} mid: {} right: {}".format(left_ir_val, mid_ir_val, right_ir_val))
    

    ir_th_val = 900
    distance = 1000
    # Process sensor date here.
    #wheel2 wheel4 right side index 1,3
    #wheel1 wheel3 left side index 0,2
    if left_ir_val<ir_th_val and right_ir_val<ir_th_val and mid_ir_val>=ir_th_val:
        #error = ((left_ir_val - ir_th_val) + (right_ir_val - ir_th_val))/2
        error = 0
        balance = pid(error)
        set_speed(speed, balance)
        
    if left_ir_val<ir_th_val and right_ir_val>=ir_th_val and mid_ir_val>=ir_th_val:
        #goes right
        error = distance/2
        balance = pid(error)
        set_speed(speed, balance)
    
    if left_ir_val>=ir_th_val and right_ir_val<ir_th_val and mid_ir_val>=ir_th_val:
        #goes left
        error = -distance/2
        balance = pid(error)
        set_speed(speed, balance)
    
    if left_ir_val>=ir_th_val and right_ir_val<ir_th_val and mid_ir_val<ir_th_val:
        #goes left
        error = -distance
        balance = pid(error)
        set_speed(speed, balance)
    
    if left_ir_val<ir_th_val and right_ir_val>=ir_th_val and mid_ir_val<ir_th_val:
        #goes right
        error = distance
        balance = pid(error)
        set_speed(speed, balance)
    
    if left_ir_val<ir_th_val and right_ir_val<ir_th_val and mid_ir_val<ir_th_val:
        #rotates
        error = 0
        balance = pid(error)
        set_speed(-speed, balance)
        
