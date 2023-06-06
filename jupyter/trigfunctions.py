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

def distance(src, target):
    (x1, y1) = src
    (x2, y2) = target
    dx = x2 - x1
    dy = y2 - y1

    return math.sqrt(dx * dx + dy * dy)

