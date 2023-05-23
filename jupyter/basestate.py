import math

class BaseState:
    
    def normalizerad(self, x):
        if x > 0:
            return x % math.pi
        return math.pi + (x % math.pi)

    def signedAngle(self, a, b):
        a = math.degrees(self.normalizerad(b)) - math.degrees(self.normalizerad(a))
        if a > 180.0:
            a -= 360.0 
        if a < -180.0:
            a += 360.0 
        return a    
    
    def process(self, controller):
        pass
