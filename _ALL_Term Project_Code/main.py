'''!@file       main.py
    @brief      This is the main file that sends data to stepper.py and
                task_motor.py to control the dc and the two stepper motors
    @details    Here we parse through an HPGL file that we created by drawing
                a shape in inkscape. The parse function creates three arrays for
                x and y location as well as pen up/down commands. From there, 
                these arrays are put into a queue that is read in the motor_task function
                which further interpolates our data to create bytearrays that are
                to be pushed to our stepper.py file
    @author     Miles Ibarra
    @author     Matteo Gozzin
    @date       06/09/2022
'''
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
import motor as moe
import utime
import math
from math import sqrt

##global variable that helps determine when our robot is meant to draw a line or turn
## so that we can change the wheel configuration (allows the wheel to run forwards or backwards depending on line or turn)
idx = 0

def motor_task():
    '''@brief    Drives the dc and stepper motors.
       @details  Reads from the command and loocation queues to create byte arrays that will be sent to the Stepper.py file.
                 Interprets the commands array and adjusts PWM of the dc motor to raise and lower the pen
       
    '''
    global idx
    state = 0
    ## this gives each line/turn time to fully complete before moving onto the next line/turn
    time.sleep(3)
    if state == 0:
        ##reads the next command in the queue for our pen motion
        com = command_queue.get()
        if com == 1:
            print('down')
            ##this will move the DC motor downwards so that the pen will be able to draw
            motor2.set_duty_cycle(-30)
            state = 1
            
        elif com == 0:
            print('up')
            ##this will move the DC motor upawrds so that the pen wont touch the ground when ew dont want it to
            motor2.set_duty_cycle(40)
            state = 1
    if state == 1:
        print('state 1')
        ##creates empty byte array to house our next xtarget value
        TX = bytearray(4)
        
        ##reads the next value in the queue to be placed in the byte array
        tar = x_queue.get()
        ##convers the decimal number to binary and takes out thee leading "0b"
        number = bin(tar)[2:]

        ##checks if our byte is longer than 8 bits
        if len(number) >8:
            maxB = 256
            ## seperates our larger than 8 bits byte into the first 8 bytes and the remaining ones
            first8 = number[len(number)-8:len(number)]
            ##these are the remaining bits not in the first 8
            second8 = number[:len(number)-8]
            ##converts our binay back to decimal
            ValF8 = int(first8,2)
            ##sets the values of our xtarget bytearay to be pushed to the steppers
            TX[3] = ValF8
            TX[2] = int((tar-ValF8)/maxB)
            TX[1] =0
            print(TX)
            state = 2
        else:
            ##if the byte is not longer than 8 bits, simply adds the original decimal value to the bytearray
            TX[3] = tar
            print(TX)
            state = 2
            
    if state == 2:
        ##pins to enable the motors and chip select so that we can control which stepper we want to move
        EN1 = Pin(Pin.cpu.B0, mode=Pin.OUT_PP, value=1)
        EN2 = Pin(Pin.cpu.C2, mode=Pin.OUT_PP, value=1)
        nCS1 = pyb.Pin(pyb.Pin.cpu.C0, mode=pyb.Pin.OUT_PP, value=1)
        nCS2 = pyb.Pin(pyb.Pin.cpu.C3, mode=pyb.Pin.OUT_PP, value=1)
        ##instantiates spi communicatin
        spi = SPI(2, SPI.MASTER, baudrate=100000, polarity=1, phase=1, crc=None)
        ##these two registers are held constant and direct which direction the steppers rotate in, helpful for curves and turns
        invConfig = bytearray([0b01101000,
                               0b00000000,
                               0b00000000,
                               0b00110000])

        regConfig = bytearray([0b01101000,
                               0b00000000,
                               0b00000000,
                               0b00100000])
        
        print('state 2') 
        ##the first push will zero the stepper direction
        ##this acts as a filter to decide whena turn is coming or when a line is comming and will change the 
        ##configuratin of the stepper motors
        if ((idx%2) == 1) or (idx == 0):
            print('line')
            ##pushes the xtarget bytearray to the stepper.py file    
            l = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, invConfig, TX, TX)
            idx += 1
            print('idx',idx)
            state = 0
            motor_task()
        elif (idx%2) == 0:
            print('turn')
            t = Stepper.TMC4210(nCS1, nCS2, spi, EN1, EN2, regConfig, regConfig, TX, TX)
            idx +=1
            print('idx',idx)
            motor_task()



def task_parse():
    '''@brief    Parses through the HPGL file and splits up coordinate locations and pen commands
       @details  Reads the given HPGL file and appends the data into 3 various arrays. 
                 The arrays are then converted to inches so that we can perform easier math on how many steps it takes to 
                 get our desired location
       
    '''
    ##holds the  decimal value to be put into a byte array
    dec_val  = 0
    
    ##various lists that hold data to be modified
    data = []
    commands = []
    pen = []
    x_array = []
    y_array = []
    ##reads the HPGL file and splits out the ;
    with open("Car.hpgl","r") as file:
        line = file.read()
        data = line.split(';')
    ##splits out the commas
    cmd = len(data)*[0]
    for i in range(len(data)):
        cmd[i] = data[i].strip(',').split(',')
        ##checks for PU and PD in the data list
        ## if a PD or PU is found, it will take the first 2 places as a command  variable and everything after will go into the xarray
        ## this is slightly  confusing to describe, but our HPGL file starts each PU or PD command like this: PD0 or PU7500 etc
        ##since thereare no commas to splice out we have to only take the first two letters ("PD" or "PU") to  put into the command array and the rest into the location arrays
        if (cmd[i][0] == 'PU' or 'PD') and (len(cmd[i])>1):
            #x_queue.put(int(cmd[i][0][2:]))
            
            x_array.append(cmd[i][0][2:])
            for k in range(((len(cmd[i]))-1)/2):
                commands.append(cmd[i][0][0:2])
        else:
            pass
        for j in range(len(cmd[i])):
            if (j%2 == 1):
                #y_queue.put(int(cmd[i][(j)]))
                y_array.append(cmd[i][(j)])
            if (j>1) and (j%2 == 0):
                #x_queue.put(int(cmd[i][(j)]))
                x_array.append(cmd[i][(j)])
    ##converts the PD and PU commands into ones and zeros so that we can put them in a queue
    for m in range(len(commands)):
        if commands[m] == 'PD':
            command_queue.put(1)
            pen.append(1)
        if commands[m] == 'PU':
            command_queue.put(0)
            pen.append(0)
    ##simplifies the coordinate commands into inches
    x_vals = [int(x)/1000 for x in x_array]
    y_vals = [int(y)/1000 for y in y_array]
    
    #print('pen', pen, len(commands))
    
    ##checks the y and x values at each location to then calculate how many steps needed to get that distance or perform a turn
    for i in range(len(x_vals)):
        if i == 0:
            dec_val += 0
            x_queue.put(dec_val)
            print(dec_val)
            
        elif (i == 2 or i == 4  or i ==6):
            dec_val += 112
            x_queue.put(dec_val)
            print(dec_val)
        elif (i == 8 or i == 10 or i ==12 or i == 14):
            dec_val += 168
            x_queue.put(dec_val)
            print(dec_val)
            
        elif ((x_vals[i] - x_vals[i-1]) !=0) or ((y_vals[i] - y_vals[i-1]) != 0):
            ##pythagorean theorem
            dec_val += int((sqrt((abs(x_vals[i-1]-x_vals[i])*abs(x_vals[i-1]-x_vals[i]))+(abs(y_vals[i-1]-y_vals[i])*(abs(y_vals[i-1]-y_vals[i])))))*conversion)
            x_queue.put(dec_val)
            print(dec_val)
    
    
if __name__ == '__main__':
    ##starts the clock for spi communication
    Stepper.TMC4210.Clockinit()
    print('clock')
    dec_val  = 0
    TX = bytearray(4)
    TX[0] = 0b00110010
    
    conversion = 128/7.5

    micropython.alloc_emergency_exception_buf(100)    
    ##instantiates the dc motor pins
    motor2 = moe.MotorDriver(pyb.Pin.cpu.C1, pyb.Pin.cpu.A0, pyb.Pin.cpu.A1, 5) 
    ##enables the dc motor
    motor2.enable()
    ##sets duty cycle of the motor
    motor2.set_duty_cycle(0)
    
    
    ##Stepper pins

    EN1 = Pin(Pin.cpu.B0, mode=Pin.OUT_PP, value=1)
    EN2 = Pin(Pin.cpu.C2, mode=Pin.OUT_PP, value=1)
    nCS1 = pyb.Pin(pyb.Pin.cpu.C0, mode=pyb.Pin.OUT_PP, value=1)
    nCS2 = pyb.Pin(pyb.Pin.cpu.C3, mode=pyb.Pin.OUT_PP, value=1)
## spi instantiation
    spi = SPI(2, SPI.MASTER, baudrate=100000, polarity=1, phase=1, crc=None)
    
    ##various queues to hold cordinate and command data
    stopFlag = task_share.Share ('h', thread_protect = False, name = "stop")
    GateFlag = task_share.Share ('h', thread_protect = False, name = "gate")
    x_queue = task_share.Queue('H', 17, thread_protect = False, 
                                 overwrite = False, name = "Queue x")
    y_queue = task_share.Queue('H', 17, thread_protect = False, 
                                 overwrite = False, name = "Queue y")
    command_queue = task_share.Queue('H', 17, thread_protect = False, 
                                 overwrite = False, name = "Queue commands")
    
    stopFlag.put(0)
    GateFlag.put(0)
    ##runs the functions above
    task_parse() 
    motor_task()
 
    
