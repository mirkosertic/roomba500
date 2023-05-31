import math

def normalizerad(x):
    if x > 0:
        return x % math.pi
    return math.pi + (x % math.pi)

def signedangle(a, b):
    a = math.degrees(normalizerad(b)) - math.degrees(normalizerad(a))
    if a > 180.0:
        a -= 360.0
    if a < -180.0:
        a += 360.0
    return a
