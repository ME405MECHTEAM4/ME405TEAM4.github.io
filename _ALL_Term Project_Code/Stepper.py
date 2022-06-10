import pyb
from pyb import SPI
from pyb import Pin, Timer
import time
# import cotask
# import task_share
# import micropython
# import gc

#spi = SPI(2, SPI.MASTER, baudrate=100000, polarity=1, phase=1, crc=None)

class TMC4210:

    def __init__(self, nCS, NCS, spi, EN, En, config1, config2, xtar1, xtar2):
        #self.spi = SPI(2, SPI.MASTER, baudrate=100000, polarity=1, phase=1, crc=None)
        self.nCS = nCS
        self.NCS = NCS
        self.spi = spi
        self.EN = EN
        self.En =En
        self.config1 = config1
        self.config2 = config2
        self.xtar1 = xtar1
        self.xtar2 = xtar2
        self.TX_array = bytearray(4) ##for motor 1
        self.TX2_array = bytearray(4) ##for motor 2
        self.RX_array = bytearray(4) ##for motor 1
        self.RX2_array = bytearray(4) ##for motor 2
        
        self.tr_ad = 0b01110011
        
        self.vmin_ad= 0b00000100

        self.vmin_r3 = 0b00000001
        
        self.vmax_ad= 0b00000110

        self.vmax_r3 = 0b00000010
        
        self.PdivRdiv_ad = 0b00011000
        self.PdivRdiv_r2 = 0b00110011 #5 for each
        
        self.Amax_ad = 0b00001100
        self.Amax_r3 = 0b00000010
        
        self.PmulPdiv_ad = 0b00010010
        self.PmulPdiv_r2 = 0b10000000
        self.PmulPdiv_r3 = 0b00001000
        
        self.rampmode_ad = 0b00010100
        self.rampmode_r1 = 0b00000001
        self.rampmode_r2 = 0b00001000
        self.rampmode_r3 = 0b00000000
        
        self.rs_ad = 0b01111110
        self.rs_r1 = 0b00100000        
        
        #steps to start motor
        self.enable()
        self.type_reg()
        self.if_config()
        self.v_min()
        self.v_max()
        self.PdivRdiv()
        self.Amax()
        self.Pmul_Pdiv()
        self.mode()
        self.ref_switch()
        self.x_target()
            
 
    def Clockinit ():
        print('clock setup')   
        PA0 = Pin(Pin.cpu.A0, mode = Pin.OUT_PP)
        tim = Timer(2, period = 3, prescaler = 0)
        
        CLK = Timer(2, period=3, prescaler=0).channel(2, mode=Timer.PWM, pin=Pin.cpu.B3, pulse_width=2)
    
    def enable(self):
        print('enable')
        self.EN.low()
        self.En.low()
    
    def type_reg(self):
        print('typre')
        #print('type_reg')
        
        self.TX_array[0] = self.tr_ad
        self.TX_array[1] = 0
        self.TX_array[2] = 0
        self.TX_array[3] = 0
        
        for idx,byte in enumerate(self.TX_array): print(f"b{idx}: {byte:#010b} {byte:#04x}")
        
        self.nCS.low()
        self.NCS.low()
        self.spi.send_recv(self.TX_array, self.RX_array)
        self.nCS.high()
        self.NCS.high()
        
        print('type reg')
        #for idx,byte in enumerate(self.RX_array): print(f"b{idx}: {byte:#010b} {byte:#04x}")
  

    def if_config(self):
        print('    ')
        print('if_config')
        
        ##config motor 1
        #self.TX_array = self.config1
        
        ##config motor 2 
        #self.TX2_array = self.config2
        
        for idx,byte in enumerate(self.config1): print(f"b{idx}: {byte:#010b} {byte:#04x}")
        for idx,byte in enumerate(self.config2): print(f"b{idx}: {byte:#010b} {byte:#04x}")
        
        self.nCS.low()
        print('yo')
        self.spi.send_recv(self.config1)
        self.nCS.high()
        
        self.NCS.low()
        self.spi.send_recv(self.config2)
        self.NCS.high()
        
        print('    ')
        for idx,byte in enumerate(self.RX_array): print(f"b{idx}: {byte:#010b} {byte:#04x}")
    
    def v_min(self):
        #print('    ')
        #print('vmin')
        
        self.TX_array[0] = self.vmin_ad
        self.TX_array[1] = 0
        self.TX_array[2] = 0
        self.TX_array[3] = self.vmin_r3
        
        #for idx,byte in enumerate(self.TX_array): print(f"b{idx}: {byte:#010b} {byte:#04x}")
        
        self.nCS.low()
        self.NCS.low()
        self.spi.send_recv(self.TX_array)
        self.nCS.high()
        self.NCS.high()


    def v_max(self):
        #print('    ')
        #print('vmax')
        
        self.TX_array[0] = self.vmax_ad
        self.TX_array[1] = 0
        self.TX_array[2] = 0
        self.TX_array[3] = self.vmax_r3
        
        #for idx,byte in enumerate(self.TX_array): print(f"b{idx}: {byte:#010b} {byte:#04x}")
        
        self.nCS.low()
        self.NCS.low()
        self.spi.send_recv(self.TX_array)
        self.nCS.high()
        self.NCS.high()
        
    def PdivRdiv(self):
        #print('    ')
        #print('pulse & ramp div')
        
        self.TX_array[0] = self.PdivRdiv_ad
        self.TX_array[1] = 0
        self.TX_array[2] = self.PdivRdiv_r2
        self.TX_array[3] = 0
        
        #for idx,byte in enumerate(self.TX_array): print(f"b{idx}: {byte:#010b} {byte:#04x}")
        
        self.nCS.low()
        self.NCS.low()
        self.spi.send_recv(self.TX_array)
        self.nCS.high()
        self.NCS.high()
    
    def Amax(self):
        #print('    ')
        #print('A max')
        
        self.TX_array[0] = self.Amax_ad
        self.TX_array[1] = 0
        self.TX_array[2] = 0
        self.TX_array[3] = self.Amax_r3
        
        #for idx,byte in enumerate(self.TX_array): print(f"b{idx}: {byte:#010b} {byte:#04x}")
        
        self.nCS.low()
        self.NCS.low()
        self.spi.send_recv(self.TX_array)
        self.nCS.high()
        self.NCS.high()
    
    def Pmul_Pdiv(self):
        #print('    ')
        #print('P mul & div')
        
        self.TX_array[0] = self.PmulPdiv_ad
        self.TX_array[1] = 0
        self.TX_array[2] = self.PmulPdiv_r2
        self.TX_array[3] = self.PmulPdiv_r3
        
        #for idx,byte in enumerate(self.TX_array): print(f"b{idx}: {byte:#010b} {byte:#04x}")
        
        self.nCS.low()
        self.NCS.low()
        self.spi.send_recv(self.TX_array)
        self.nCS.high()
        self.NCS.high()
        
    def mode(self):
        #print('    ')
        #print('mode')
        
        self.TX_array[0] = self.rampmode_ad
        self.TX_array[1] = self.rampmode_r1
        self.TX_array[2] = self.rampmode_r2
        self.TX_array[3] = self.rampmode_r3
        
        #for idx,byte in enumerate(self.TX_array): print(f"b{idx}: {byte:#010b} {byte:#04x}")
        
        self.nCS.low()
        self.NCS.low()
        self.spi.send_recv(self.TX_array)
        self.nCS.high()
        self.NCS.high()
    
    def ref_switch (self):
        #print('    ')
        #print('Ref Switch')
        
        self.TX_array[0] = self.rs_ad
        self.TX_array[1] = self.rs_r1
        self.TX_array[2] = 0
        self.TX_array[3] = 0
        
        #for idx,byte in enumerate(self.TX_array): print(f"b{idx}: {byte:#010b} {byte:#04x}")
        
        self.nCS.low()
        self.NCS.low()
        self.spi.send_recv(self.TX_array)
        self.nCS.high()
        self.NCS.high()
        
    def x_target (self):
    
        print('    ')
        print('x target')
        #motor 1 first motion
        self.TX_array = self.xtar1
        
        ##motor 2 first motion
        self.TX2_array = self.xtar2       
    
        for idx,byte in enumerate(self.TX_array): print(f"b{idx}: {byte:#010b} {byte:#04x}")
        for idx,byte in enumerate(self.TX2_array): print(f"b{idx}: {byte:#010b} {byte:#04x}")
        self.NCS.low()
        self.spi.send_recv(self.TX2_array)
        self.NCS.high()
        
        self.nCS.low()
        self.spi.send_recv(self.TX_array)
        self.nCS.high()
