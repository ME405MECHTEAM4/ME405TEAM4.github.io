#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 31 20:35:10 2022

@author: matteogozzini
"""

"""@file           chassis_driver.py
   
"""

class chassis:
    '''@brief       Interface with chassis system
       @details     Contains basic kinematics for two wheeled device
    '''
    
    def __init__(self, Rc, Rw,  GearinR, encoderR, overalltune, r, l):
        '''
        @brief              Constructor for chassis
        @param Rc           Radius of chassis to wheels
        @param Rw           Radius of wheels
        @param GearinR      Gear ratio of device
        @param encoderR     Encoder ratio of encoder to one revolution
        @param overalltune  Tuning value for slippage
        @param r            Right wheel tuning parameter
        @param l            Left wheel tuning paramter
        '''
        ## Distance from the pen POC to the center of the wheel face
        self.R_Center = Rc
        ## Wheel radius
        self.R_Wheel = Rw
        ## Gear ratio
        self.Gear_Ratio = GearinR
        ## Encoder conversion factor
        self.Encoder_Ratio = encoderR
        ## Parameter to manually tune the magnitude of rotation or translation
        self.tune_t = overalltune
        ## Manual tuning parameter for left wheel
        self.left = l
        ## Manual tuning parameter for right wheel
        self.right = r
        
    def find_motor_dists(self, rotation, value):
        '''@brief       Calculates encoder value for each wheel based on command
           @return      Tuple containing encoder value for each wheel.
           @param rotation Rotational distance for wheel to travel
           @param value    Translational distance for wheel to travel
        '''
        if rotation:
            theta_m = self.tune_t*self.Encoder_Ratio*(self.R_Center*value)/(self.Gear_Ratio*self.R_Wheel)
            return (self.right*theta_m, self.left*theta_m)
        else:
            theta_m = self.Encoder_Ratio*(180*value)/(self.Gear_Ratio*self.R_Wheel*3.1415926)
            return (theta_m, -theta_m)