import math

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

    def panAxis(self, iteration:int=0, axis:int=0, limits:tuple=(0, 0), n_iterations:int=0):
        return
    
    def calculate_ramp_time(self,current_pos, target_pos, max_accel, max_velocity, ramp_divisor, pulse_divisor):
       # Apply divisors to get actual acceleration and velocity
       actual_accel = max_accel / (2 ** ramp_divisor)
       actual_velocity = max_velocity / (2 ** pulse_divisor)
    
       # Calculate total distance to move
       distance = abs(target_pos - current_pos)
    
       # Compute acceleration and deceleration distance
       accel_distance = (actual_velocity ** 2) / (2 * actual_accel)
    
       if distance < 2 * accel_distance:
           # Triangular profile (no constant velocity phase)
           total_time = 2 * math.sqrt(distance / actual_accel)
       else:
           # Trapezoidal profile
           accel_time = actual_velocity / actual_accel
           decel_time = accel_time
           constant_distance = distance - 2 * accel_distance
           constant_time = constant_distance / actual_velocity
           total_time = accel_time + constant_time + decel_time

       return total_time
    
    


rotationSequences1 = [
   (-90, -55, 500, 1000),
   (-90, 0, 500, 1000),
   (-90, 55, 500, 1000),
   (-90, 0, 500, 1000),
   (-90, -55, 500, 1000),
   (-90, 0, 500, 1000),
   (-90, 55, 500, 1000),
   (-90, 0, 500, 1000),
]
rotationSequences2 = [
   (0, -40, 500, 1000),
   (90, 40, 500, 1000),
   (-90, -40, 500, 1000),
   (90, -40, 500, 1000),
]

facingSequence = [
    (35, -35, 200, 200),
    (0, 35, 200, 200),
    (-35, -35, 200, 200),
]
facingAwaySequence = [
    (35, 35, 200, 200),
    (0, -35, 200, 200),
    (-35, 35, 200, 200),
]
Sequence1 = [
    (0, -35, 200, 200),
    (35, 35, 200, 200),
    (0, -35, 200, 200),
]
Sequence2 = [
    (-35, 35, 200, 200),
    (0, -35, 200, 200),
    (35, 0, 200, 200),
]
flat = [
    (0, 0, 200, 200),
    (35, 0, 200, 200),
    (0, 0, 200, 200),
]
demoSequence = [
    facingSequence,
    facingAwaySequence,
    Sequence1,
    Sequence2,
    flat
]
