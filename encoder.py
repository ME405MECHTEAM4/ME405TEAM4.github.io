#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 31 20:35:10 2022

@author: matteogozzini
"""

"""@file           encoder_elwell_mccue.py
   @brief          A driver for reading from the Quadrature Encoders
   @details        The functions of this driver allow one to "get", "set", and "find" the encoder delta.
   
                   The driver creates a 16-bit timer, whose channels correlate ticks to encoder rotation. The driver also
                   has a method that is used to return encoder position and velocity to the encoder task.
   @author         Clayton Elwell
   @author         Tyler McCue
   @date           January 27, 2022
"""
import pyb

class EncoderDriver:
    '''@brief       Interface with quadrature encoder
       @details     Contains the basic functionality of an encoder that will be sent to the encoder task upon user request.
    '''

    
    def __init__(self, pin1, pin2, timerID):
        
        '''@brief       Constructs an encoder object and assigns the correct pins and timer.
           @details     Creates a 16-bit timer whose number is defined in the encoder task. The timer channels and pins are defined 
                        in the encoder task.
           @param       pin1    Pin to be used with the encoder channel 1
           @param       pin2    Pin to be used with the encoder channel 2
           @param       timerID  Timer channel to be used with the previously assigned pins.
        '''
        ## Total rotational position of the encoder, measured in ticks.
        self.position = 0
        ## Encoder position correlated to the update prior to the most recent update.
        self.position1 = 0
        ## Encoder position correlated to the most recent update
        self.position2 = 0
        ## Difference (in ticks) between the most recent encoder update and the update prior to the most recent update.
        self.delta = 0
        
        ##  Timer for CH1 and CH2 pins
        self.timX = pyb.Timer(timerID, prescaler = 0, period = 65535)
        ## Assigning the pin1 parameter to the STM32L476RG channel 1
        self.timX.channel(1, pyb.Timer.ENC_AB, pin=pin1)
        ## Assigning the pin2 parameter to the STM32L476RG channel 2
        self.timX.channel(2, pyb.Timer.ENC_AB, pin=pin2)
        
    def update(self):
        '''@brief       Updates encoder position and delta. Also sets the encoder position and returns the current encoder position.
           @return      True rotational position of the encoder.       
        '''
        self.position1 = self.position2
        self.position2 = self.timX.counter()
        self.delta = self.get_delta()
        
        if(self.delta < -32768):
            self.position += (self.delta + 65536)
        elif(self.delta > 32768):
            self.position += (self.delta - 65536)
        else: 
            self.position += self.delta
        return self.position        

    def get_position(self):
        '''@brief      Returns encoder position.
           @return     The position of the encoder shaft.
        '''
        
        return self.position

    def set_position(self, pos):
        '''@brief       Set the encoder position to the value of parameter "pos".
            
           @param pos   The new position of the encoder shaft.
        '''
        self.position = pos
        self.delta = 0
        self.timX.counter(0)
        self.position1 = 0
        self.position2 = 0
        
        
    
    def get_delta(self):
        '''@brief      Returns the difference (in ticks) between the two most recent encoder updates.
           
           @return     The change in position of the encoder shaft between
                       the two most recent updates.
        '''
        return((self.position2 - self.position1))

# if __name__ == '__main__':
#     
#     enc1 = EncoderDriver(pyb.Pin.cpu.B6, pyb.Pin.cpu.B7, 4)
#     
#     enc2 = EncoderDriver(pyb.Pin.cpu.C6, pyb.Pin.cpu.C7, 8)
#     
#     
#     while True:
#         try:
#             print('{:}, {:}'.format(enc1.update(), enc2.update()))
#             
#         except KeyboardInterrupt:
#             break