import array
import utime
## Encoder conversion factor
tick2deg = 1/(256*16*4/360)

class ClosedLoop:
    '''
    @brief      Class that closes loop to control the system
    @details    Uses shared variables to get an error in the system
                which the controller then eliminates
    '''

    def __init__(self, saturation_low, saturation_high, Kp, dhigh, dlow):
        '''
        @brief          Constructor for closed loop controller
        @param saturation_low   Loweset value actuatuion can be based on system
        @param saturation_high  Highest value actuatuion can be based on system
        @param Kp               Proportional gain
        @param dhigh            Duty cycle saturation upper limit
        @param dlow             Duty cycle saturation lower limit
        '''
        ## Initial time
        self.t0 = utime.ticks_ms()
        ## creates class variable for lowest acutation value
        self.saturation_low = saturation_low
        ## creates class variable for highest acutation value
        self.saturation_high = saturation_high
        ## creates class variable for proportional gain
        self.Kp = Kp
        ## creates class variable for reference value
        self.reference = 0
        ## creates class variable for actuation
        self.actuation = 0
        ## Low dead zone attribute
        self.dead_zone_low = dlow
        ## High dead zone attribute
        self.dead_zone_high = dhigh
        ## Time array attribute for plotting
        self.tArray = array.array('f',[])
        ## Position array attribute for plotting
        self.pArray = array.array('f',[])
        ## Error array of last 10 values
        self.Error = []
        
        #self.perAvg = 100
        #self.done = True

    def run(self, feedback):
        '''
        @brief              Updates actuation value from error and gain
        @param feedback     Refers to the output of the system
        '''
        
        current_time = utime.ticks_ms() - self.t0
        
        #self.tArray.append(current_time/1000)
        #self.pArray.append(feedback*tick2deg)
        
        prev = self.actuation
        self.actuation = self.Kp*(self.reference - feedback)
        
        diff = prev - self.actuation
        if diff > 0:
           if self.actuation < self.dead_zone_high and self.actuation > self.dead_zone_low:
               self.actuation = self.dead_zone_low
        if diff < 0:
           if self.actuation > self.dead_zone_low and self.actuation < self.dead_zone_high:
               self.actuation = self.dead_zone_high
               
        if self.actuation > self.saturation_high:
            self.actuation = self.saturation_high
        elif self.actuation < self.saturation_low:
            self.actuation = self.saturation_low
        return self.actuation


    def setReference(self, speed):
        '''
        @brief              Updates reference value
        @param speed        Refers to the new reference value
        '''
        self.reference = speed

    def getReference(self):
        '''
        @brief              Gets reference value
        '''
        return self.reference

    def get_Kp(self):
        '''
        @brief              Gets proportional gain
        '''
        return self.Kp

    def set_Kp(self, new):
        '''
        @brief              Updates gain value value
        @param new          Refers to the new proportional gain
        '''
        self.Kp = new
        
    def print_data(self):
        '''@brief Prints the collected position and time data in two columns.
        '''
        #print('Time [sec], Position [deg]')
        for i in range(len(self.tArray)):
            print('{:},{:}'.format(self.tArray[i], self.pArray[i]))
        
        #print(self.tArray, self.pArray)
        
    def send_data(self):
        '''@brief Sends position and time array data when called
           @return Arrays of position and time data
        '''
        return [self.tArray, self.pArray]