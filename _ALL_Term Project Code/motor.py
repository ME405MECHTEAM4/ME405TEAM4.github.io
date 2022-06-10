#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 31 20:35:09 2022

@author: matteogozzini
"""

''' @file   motor_elwell_mccue.py

'''

import pyb
import time

class MotorDriver:
    '''@brief    Module that allows one to interface with the ME 405 BLDC motors.
       @details  Instantiates pins, timers, and channels necessary for PWM control. Allows one to enable, disable, and set duty
                 cycle for the motors.
    '''
    
    def __init__(self, en_pin, in1pin, in2pin, timer):
        '''@brief    Creates pin attributes for various motor driver functions.
           @details  Creates pin attributes for PWM, and channels and timers for sending PWM signals. Also sets up
                     enable/disable pin.
           @param    en_pin Motor enabler pin
           @param    in1pin Motor power pin 1
           @param    in2pin Motor power pin 2
           @param    timer  Timer pin
           
        '''
        
        ## Pin IN1 attribute
        pinIN1 = pyb.Pin(in1pin, pyb.Pin.OUT_PP)
        
        ## Pin IN2 attribute
        pinIN2 = pyb.Pin(in2pin, pyb.Pin.OUT_PP)
        
        ## Timer attribute
        self.tim = pyb.Timer(timer, freq = 20000)
        
        ## Channel 1 attribute
        self.ch1 = self.tim.channel(1, pyb.Timer.PWM, pin=pinIN1)
        
        ## Channel 2 attribute
        self.ch2 = self.tim.channel(2, pyb.Timer.PWM, pin=pinIN2)
        
        ## Enable/Disable Pin
        self.onoff = pyb.Pin(en_pin, pyb.Pin.OUT_PP)
        
    
    def enable(self):
        '''@brief    Enables the motor
           @details  Sets the enable/disable pin to "high"
        '''
        self.onoff.high()
        time.sleep(0.25)
    
    def disable(self):
        '''@brief    Disables the motor
           @details  Sets the enable/disable pin to "low"
        '''
        self.onoff.low()
        time.sleep(0.25)
    
    def set_duty_cycle(self, duty):
        '''@brief    Sets the duty cycle for the motor
           @details  Uses the input duty cycle to send a PWM percentage to the motors. Includes saturation limits.
           @param    duty A signed integer holding the duty cycle of one voltage sent to the motor
        '''
        
        if duty >= 0:
             if duty <=100:
                
                self.ch1.pulse_width_percent(100)
                self.ch2.pulse_width_percent(100 - duty)
                
             else:
                self.ch1.pulse_width_percent(100)
                self.ch2.pulse_width_percent(0)
        elif duty <0:
             if duty >= -100:
                self.ch2.pulse_width_percent(100)
                self.ch1.pulse_width_percent(100 + duty)
             
             else:
                
                self.ch2.pulse_width_percent(100)
                self.ch1.pulse_width_percent(0)
        
        
        #print('Setting duty cycle to ' + str(duty))
        
        
# if __name__ == '__main__':
#     
#     motor = MotorDriver(pyb.Pin.cpu.A10, pyb.Pin.cpu.B4, pyb.Pin.cpu.B5, 3)
#     motor.enable()