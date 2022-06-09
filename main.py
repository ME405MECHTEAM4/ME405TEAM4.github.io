import pyb
from pyb import SPI
import time
import Stepper
import micropython
import gc
import array
from pyb import Pin, ExtInt, repl_uart, UART, Timer
import cotask
import task_share
import control
import encoder as enc
import motor as moe
import utime
import chasy as ch


# def StepM():

#     while True:
#         #Stepper.TMC4210(nCS,NCS,spi, en1,en2,StepConfig.get(2),StepConfig(1),LinesArray.get(1),LinesArray.get(1))
#         yield(0)
        
def motor_task ():
    """!
    Task controls motor 1 
    """
    motor1.enable()
    while True:
        update1 = enc1.update()
        duty1 = control1.run(update1)
        motor1.set_duty_cycle(duty1)
        if abs(duty1) <= 20 and enc1.get_delta() < 5:
            share_ready1.put(1)
        if share_ready1.get() == 1 and share_ready2.get() == 1:
            share_bothR.put(1)
        yield (0)


def task_MCU2 ():
    """!
    Task controls motor 2.
    """
    motor2.enable()
    while True:
        update2 = enc2.update()
        duty2 = control2.run(update2)
        motor2.set_duty_cycle(duty2)
        if abs(duty2) <= 20 and enc2.get_delta() < 5:
            share_ready2.put(1)
        if share_ready1.get() == 1 and share_ready2.get() == 1:
            share_bothR.put(1)
        yield (0)
        
def task_interpret ():
    """!
    Task interprets commands from file to control certain motors
    """
    while True:
        if share_bothR.get() == 1:
            if share_comm.get() == 0:
                (ref1, ref2) = chass.find_motor_dists(True, share_theta.get())
                control1.setReference(ref1)
                control2.setReference(ref2)
                enc1.set_position(0)
                enc2.set_position(0)
                share_bothR.put(0)
                share_ready2.put(0)
                share_ready1.put(0)
            if share_comm.get() == 1:
            
                (ref1, ref2) = chass.find_motor_dists(False, share_dist.get())
                control1.setReference(ref1)
                control2.setReference(ref2)
                enc1.set_position(0)
                enc2.set_position(0)
                share_bothR.put(0)
                share_ready2.put(0)
                share_ready1.put(0)
            if share_comm.get() == 2:
                share_nextI.put(1)
            share_comm.put(share_comm.get() + 1)
            if share_comm.get() > 2:
                share_comm.put(0)
        yield(0)
            

def task_parse ():
    """!
    Task parses file for commands
    """
    print('yo')
    with open(r"code.txt", "r") as c:
        while True:
            if share_nextI.get() == 1:
                commands = c.readline()
                #print('commands',commands)
                if "st" in commands:
                    share_stop.put(1)
                else:    
                    if len(commands) > 0:
                         commands = commands.split("_")
                         share_theta.put(float(commands[0][1:]))
                         share_dist.put(float(commands[1][1:]))
                         share_pen.put(float(commands[2][1:]))
            share_nextI.put(0)
            yield(0)
        
def task_parse():
    data = []
    commands = []
    x_array = []
    y_array = []
    with open("Car.hpgl","r") as file:
        line = file.read()
        data = line.split(';')
    cmd = len(data)*[0]
    for i in range(len(data)):
        cmd[i] = data[i].strip(',').split(',')
        if (cmd[i][0] == 'PU' or 'PD') and (len(cmd[i])>1):
            x_array.append(cmd[i][0][2:])
            for k in range(((len(cmd[i]))-1)/2):
                commands.append(cmd[i][0][0:2])
                #x_array.append(cmd[i][0][2:])
                #print(x_array)
        else:
            pass
        for j in range(len(cmd[i])):
            if (j%2 == 1):
                y_array.append(cmd[i][(j)])
            if (j>1) and (j%2 == 0):
                x_array.append(cmd[i][(j)])

    x_vals = [int(x)/1000 for x in x_array]
    y_vals = [int(y)/1000 for y in y_array]
    print('x',x_vals, len(x_vals))
    print('y',y_vals, len(y_vals))
    print('commands', commands, len(commands))
    while True:
        if "PD" in commands:
            share_stop.put(1)
        else:
            if len(commands) > 0:
                commands = commands.split("_")
                share_theta.put(float(commands[0][1:]))
                share_dist.put(float(commands[1][1:]))
                share_pen.put(float(commands[2][1:]))
                share_nextI.put(0)
            
                
            
        yield(0)
        

    
if __name__ == '__main__':
    
 
    
    micropython.alloc_emergency_exception_buf(100)
    
    ## Right wheel tuning parameter
    right = 1.01
    ## Left wheel tuning parameter
    left = 1.01
    ## Chassis tuning parameter
    tune = 1.02
    ## Pen POC to wheel distance
    center = 3.03
    ## Wheel radius
    wheel = 2.53/2
    ## Gear ratio
    gear = 16/71
    ## Encoder conversion factor
    encoder = (256*16*4/360)

    ##Driver for chassis
    chass = ch.chassis(center, wheel, gear, encoder, tune, right, left)
    
    
    
    ## Driver object for first motor
    motor1 = moe.MotorDriver(pyb.Pin.cpu.A10, pyb.Pin.cpu.B4, pyb.Pin.cpu.B5, 3)
    
    ## Driver object for second motor
    motor2 = moe.MotorDriver(pyb.Pin.cpu.C1, pyb.Pin.cpu.A0, pyb.Pin.cpu.A1, 5)
    
    ## Driver object for first encoder
    enc1 = enc.EncoderDriver(pyb.Pin.cpu.B6, pyb.Pin.cpu.B7, 4)
    ## Driver object for second encoder
    enc2 = enc.EncoderDriver(pyb.Pin.cpu.C6, pyb.Pin.cpu.C7, 8)
    
    
    
    enc1.set_position(0)
    enc2.set_position(0)
    
    ## Controller object for first motor
    control1 = control.ClosedLoop(-100, 100, .009, 0, 0)
    ## Controller object for second motor
    control2 = control.ClosedLoop(-100, 100, .009, 0, 0)
    
    
    
    ##Stepper pins
    Stepper.TMC4210.Clockinit()
    EN1 = Pin(Pin.cpu.B0, mode=Pin.OUT_PP, value=1)
    EN2 = Pin(Pin.cpu.C2, mode=Pin.OUT_PP, value=1)
    nCS1 = pyb.Pin(pyb.Pin.cpu.C0, mode=pyb.Pin.OUT_PP, value=1)
    nCS2 = pyb.Pin(pyb.Pin.cpu.C3, mode=pyb.Pin.OUT_PP, value=1)


    spi = SPI(2, SPI.MASTER, baudrate=100000, polarity=1, phase=1, crc=None)
    ##SCK = Pin(Pin.cpu.B13, mode = Pin.OUT_PP)
    ##MOSI = Pin(Pin.cpu.B14, mode = Pin.OUT_PP)
    ##MISO = Pin(Pin.cpu.B15, mode = Pin.OUT_PP)
    
    task1 = cotask.Task (motor_task, name = 'Task_Motor_1', priority = 3, 
                         period = 25, profile = True, trace = False)
    ## Motor 2 Task
    task2 = cotask.Task (task_MCU2, name = 'Task_Motor_2', priority = 3, 
                         period = 25, profile = True, trace = False)
    ## Interpreter Task
    task3 = cotask.Task (task_interpret, name = 'Task_Interpreter', priority = 2, 
                         period = 500, profile = True, trace = False)
    ## Parser Task
    task4 = cotask.Task (task_parse, name = 'Task_Parser', priority = 1, 
                         period = 500, profile = True, trace = False)
    
    cotask.task_list.append (task1)
    cotask.task_list.append (task2)
    cotask.task_list.append (task3)
    cotask.task_list.append (task4)
    
    # Create a shares for tasks
    ## Share to ensure motor 1 is at the correct position
    share_ready1 = task_share.Share ('f', thread_protect = False, name = "Ready1")
    ## Share to ensure motor 2 is at the correct position
    share_ready2 = task_share.Share ('f', thread_protect = False, name = "Ready2")
    ## Share to ensure both motors are at the correct position
    share_bothR = task_share.Share ('f', thread_protect = False, name = "Ready2")
    ## Share storing commands between PC and Nucleo
    share_comm = task_share.Share ('f', thread_protect = False, name = "Command")
    ## Share storing next instruction for plotter tool to execute
    share_nextI = task_share.Share ('f', thread_protect = False, name = "Stop")
    
    #share_c1val = task_share.Share ('f', thread_protect = False, name = "Val1")
    
    #share_c2val = task_share.Share ('f', thread_protect = False, name = "Val2")
    ## Current controller gain value
    share_currC = task_share.Share ('f', thread_protect = False, name = "CurrentCommand")
    ## Rotational distance for motors to travel
    share_theta = task_share.Share ('f', thread_protect = False, name = "ThetaVal")
    ## Wheel radius shared value
    share_dist = task_share.Share ('f', thread_protect = False, name = "RadiusVal")
    ## Boolean indicating whether pen should be up or down
    share_pen = task_share.Share ('f', thread_protect = False, name = "PenVal")
    ## Stores value that stops the plotter device
    share_stop = task_share.Share ('f', thread_protect = False, name = "Stop")
    
    
    share_ready1.put(0)
    share_ready2.put(0)
    share_currC.put(0)
    share_stop.put(0)
    share_nextI.put(1)
    share_comm.put(0)
    share_bothR.put(1)
    

    # Create the tasks. If trace is enabled for any task, memory will be
    # allocated for state transition tracing, and the application will run out
    # of memory after a while and quit. Therefore, use tracing only for 
    # debugging and set trace to False when it's not needed
    ## Motor 1 Task
    

    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect ()
    
    while share_stop.get() == 0:
        cotask.task_list.pri_sched()
        
    motor1.set_duty_cycle(0)
    motor2.set_duty_cycle(0)


    invConfig = bytearray([0b01101000,
                           0b00000000,
                           0b00000000,
                           0b00110000])

    regConfig = bytearray([0b01101000,
                           0b00000000,
                           0b00000000,
                           0b00100000])

    #StepConfig.put(invConfig, regConfig)
    #LinesArray.put(xtar0, xtar1, turn1, xtar2, turn2, xtar3, turn3, xtar4, turn4, xtar5, turn5, xtar6, turn6, xtar7)



   ##reset
    xtar0 = bytearray([0b00000000,
                       0b00000000,
                       0b00000000,
                       0b00000000])

    ##forward line 1 128
    xtar1 = bytearray([0b00000000,
                       0b00000000,
                       0b00000000,
                       0b10000000])
    ##turn 1 240
    turn1 = bytearray([0b00000000,
                       0b00000000,
                       0b00000000,
                       0b11110000])

    ##straight line 2 368
    xtar2 = bytearray([0b00000000,
                       0b00000000,
                       0b00000001,
                       0b01110000])
    ##turn 2 480
    turn2 = bytearray([0b00000000,
                       0b00000000,
                       0b00000001,
                       0b11100000])
    ##line 3 608
    xtar3 = bytearray([0b00000000,
                       0b00000000,
                       0b00000010,
                       0b01100000])
    ##turn 3 720
    turn3 = bytearray([0b00000000,
                       0b00000000,
                       0b00000010,
                       0b11010000])
    ##line 4 848
    xtar4 = bytearray([0b00000000,
                       0b00000000,
                       0b00000011,
                       0b01010000])
    ##turn 4 960
    turn4 = bytearray([0b00000000,
                       0b00000000,
                       0b00000011,
                       0b11000000])
    ##turn 4.1 1018
    turn41 = bytearray([0b00000000,
                       0b00000000,
                       0b00000011,
                       0b11111010])
    
    ##line 5 (diag) 1146
    xtar5 = bytearray([0b00000000,
                       0b00000000,
                       0b00000100,
                       0b01111010])
    ##line 51 (diag) 1193
    xtar51 = bytearray([0b00000000,
                       0b00000000,
                       0b00000100,
                       0b10101001])
    ##turn 5 1305
    turn5 = bytearray([0b00000000,
                       0b00000000,
                       0b00000101,
                       0b00011001])
    ##turn 51 1363
    turn51 = bytearray([0b00000000,
                       0b00000000,
                       0b00000101,
                       0b01010011])
    
    ##line 6 1491
    xtar6 = bytearray([0b00000000,
                       0b00000000,
                       0b00000101,
                       0b11010011])
    ##turn 6 1603
    turn6 = bytearray([0b00000000,
                       0b00000000,
                       0b00000110,
                       0b01000011])
    ##turn 61 1661
    turn61 = bytearray([0b00000000,
                       0b00000000,
                       0b00000110,
                       0b01111101])

    ##line 7 1789
    xtar7 = bytearray([0b00000000,
                       0b00000000,
                       0b00000110,
                       0b11111101])
    ##line 71 1828
    xtar71 = bytearray([0b00000000,
                       0b00000000,
                       0b00000111,
                       0b00100100])


    ## Hard Code
    print('zero')
    Tzero = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, invConfig, xtar0, xtar0)
    time.sleep(5)
    print('line1')
    l1 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, invConfig, xtar1, xtar1)
    time.sleep(2)
    print('turn1')
    t1 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, regConfig, turn1, turn1)
    time.sleep(2)
    print('line2')
    l2 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, invConfig, xtar2, xtar2)
    time.sleep(2)
    print('turn2')
    t2 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, regConfig, turn2, turn2)
    time.sleep(2)    
    l3 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, invConfig, xtar3, xtar3)
    time.sleep(2)
    t3 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, regConfig, turn3, turn3)
    time.sleep(2)
    l4 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, invConfig, xtar4, xtar4)
    time.sleep(2)
    t4 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, regConfig, turn4, turn4)
    time.sleep(2)
    t41 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, regConfig, turn41, turn41)
    time.sleep(2)
    
    l5 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, invConfig, xtar5, xtar5)
    time.sleep(2)
    l51 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, invConfig, xtar51, xtar51)
    time.sleep(2)
    t5 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, regConfig, turn5, turn5)
    time.sleep(2)
    t51 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, regConfig, turn51, turn51)
    time.sleep(2)
    l6 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, invConfig, xtar6, xtar6)
    time.sleep(2)
    t6 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, regConfig, turn6, turn6)
    time.sleep(2)
    t61 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, regConfig, turn61, turn61)
    time.sleep(2)
    print('turn6')
    l7 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, invConfig, xtar7, xtar7)
    l71 = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, invConfig, xtar71, xtar71)
    time.sleep(2)
    print('line 7')
         
    

    motor1.disable()
    motor2.disable()
    
   

    






#     task1 = cotask.Task (StepM, name = 'Task1', priority =1, 
#                          period = 1, profile = True, trace = False)
# 
#     task2 = cotask.Task (StepM, name = 'Task_2', priority = 2, 
#                          period = 1, profile = True, trace = False)
# 
#     task3 = cotask.Task (StepM, name = 'Task_3', priority = 3, 
#                          period = 1, profile = True, trace = False)
# 
#     task4 = cotask.Task (StepM, name = 'Task_4', priority = 4, 
#                          period = 1, profile = True, trace = False)
# 
#     task5 = cotask.Task (StepM , name = 'Task_5', priority = 5, 
#                          period = 1, profile = True, trace = False)
# 
#     task6 = cotask.Task (StepM , name = 'Task_6', priority = 6, 
#                          period = 1, profile = True, trace = False)
# 
#     task7 = cotask.Task (StepM , name = 'Task_7', priority = 7, 
#                          period = 1, profile = True, trace = False)
# 
#     task8 = cotask.Task (StepM , name = 'Task_8', priority = 8, 
#                          period = 1, profile = True, trace = False)
# 
#     task9 = cotask.Task (StepM , name = 'Task_9', priority = 9, 
#                          period = 1, profile = True, trace = False)
# 
#     task10 = cotask.Task (StepM , name = 'Task_10', priority = 10, 
#                          period = 1, profile = True, trace = False)
# 
#     task11 = cotask.Task (StepM , name = 'Task_11', priority = 11, 
#                          period = 1, profile = True, trace = False)
# 
#     task12 = cotask.Task (StepM , name = 'Task_12', priority = 12, 
#                          period = 1, profile = True, trace = False)
#     task13 = cotask.Task (StepM , name = 'Task_13', priority = 13, 
#                          period = 1, profile = True, trace = False)
#     task14 = cotask.Task (StepM, name = 'Task_14', priority =14, 
#                          period = 1, profile = True, trace = False)
# 
# 
#     cotask.task_list.append (task1)
#     cotask.task_list.append (task2)
#     cotask.task_list.append (task3)
#     cotask.task_list.append (task4)
#     cotask.task_list.append (task5)
#     cotask.task_list.append (task6)
#     cotask.task_list.append (task7)
#     cotask.task_list.append (task8)
#     cotask.task_list.append (task9)
#     cotask.task_list.append (task10)
#     cotask.task_list.append (task11)
#     cotask.task_list.append (task12)
#     cotask.task_list.append (task13)
#     cotask.task_list.append (task14)
# 
# 
#     gc.collect ()
# 
#     vcp = pyb.USB_VCP ()
#     while not vcp.any ():
#         cotask.task_list.pri_sched ()
# 
#     vcp.read ()

