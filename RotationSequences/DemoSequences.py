

# rollAngle:float = 0, tiltAngle:float = 0, speed:int = 100, velocity:int = 50
_spee = 70  #HERE
_accel = 10 #HERE

class RotationCommand():
    def __init__(self, roll:int=0, tilt:int=0, speed:int = 0, acceleration:int = 0):
        self.speed = speed
        self.acceleration = acceleration
        self.roll = roll
        self.tilt = tilt

class RotationSequenceMaker():

    def __init__(self, _speed:int = 0, _acceleration:int = 0):
        self.speed = _speed
        self.acceleration = _acceleration
    
    class Rot:
        RotateDiscLeft      =RotationCommand(-360, 0, speed=)



    





