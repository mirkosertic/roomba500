class PIDController:
    
    def __init__(self, kp, kd, ki, errorfun):
        self.lasttime = 0
        self.lasterror = 0
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.integral = 0
        self.sumerror = 0
        self.errorfun = errorfun
        
    def update(self, time, currentvalue, regulator):
        dt = time - self.lasttime
        
        if dt > 0:
            error = self.errorfun(currentvalue)
            derror = (error - self.lasterror) / dt

            self.integral += self.ki * error * dt
            
            derrivative = self.kd * derror
            
            regval = error * self.kp + derrivative + self.integral

            if regulator is not None:
                regulator(regval)
        
            self.lasttime = time
            self.lasterror = error
            self.sumerror += error
            
            return regval, error
            
        return None