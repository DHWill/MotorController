from globals import *
from MotorController import MotorController

class ControllerArm():
    def __init__(self, _rollMotor: MotorController, _tiltMotor: MotorController, _armID:int = 0):
        self.rollMotor = _rollMotor
        self.tiltMotor = _tiltMotor
        self.armID = _armID
        self.tiltTargetTargetAngle = 0
        self.rollTargetTargetAngle = 0
        self.sequencePosition = 0
        self.isSetup = False

    #This Takes ANGLE in Degrees relative to centre reference (0, 0) 
    # eg. extreme rotate left and back would be: 
    # -90, -90, speed, acceleration

    def setTargetRotationAngle(self, rollAngle:float = 0, tiltAngle:float = 0, speed:int = 100, acceleration:int = 50):
        tiltCentreAngle = self.tiltMotor.fullRotationAngle / 2.
        rollCentreAngle = self.rollMotor.fullRotationAngle / 2.
        
        rollAngle = max(min(rollAngle, rollCentreAngle), rollCentreAngle * -1)  # Clipping to 'fullRotationAngle' (distance between limit switch left/right)
        tiltAngle = max(min(tiltAngle, tiltCentreAngle), tiltCentreAngle * -1)  

        _rollAngle = rollCentreAngle + rollAngle 
        _tiltAngle = tiltCentreAngle + tiltAngle

        _tiltAngle += _rollAngle     #roll is Master, and locked on axis

        _tiltSpeed = speed
        _rollSpeed = speed
        _rollAcceleration = acceleration
        _tiltAcceleration = acceleration
        mult = 1

        currentTiltStep = self.tiltMotor.getPositionSteps()
        currentRollStep = self.rollMotor.getPositionSteps()

        tiltDistance = abs(angleToMicrostep(_tiltAngle) - currentTiltStep)
        rollDistance = abs(angleToMicrostep(_rollAngle) - currentRollStep)

        #Match Up speeds
        if(rollDistance > tiltDistance):
            mult = (tiltDistance / rollDistance)
            _tiltSpeed = speed * mult
            _tiltAcceleration = acceleration * mult
        elif(rollDistance < tiltDistance):
            mult = (rollDistance / tiltDistance)
            _rollSpeed = speed * mult
            _rollAcceleration = acceleration * mult


        self.rollMotor.setMotorTaget(int(_rollAngle), _rollSpeed, int(_rollAcceleration))
        self.tiltMotor.setMotorTaget(int(_tiltAngle), _tiltSpeed, int(_tiltAcceleration))

        #Remove This to be NON-blocking for multi arm programming
        # while((self.rollMotor.getIsPositionReached() == False) or (self.tiltMotor.getIsPositionReached() == False)):
        #     pass
    
    def setArmLimitSwitches(self, _isLimiting:bool = False):
        self.rollMotor.set_limit_switches(_isLimiting)
        self.tiltMotor.set_limit_switches(_isLimiting)

    def setupRoutine(self):
        self.rollMotor.setupDefaults()
        self.tiltMotor.setupDefaults()
        self.tiltMotor.freeMotor()  
        self.rollMotor.findHome()
        self.rollMotor.lockMotor()      
        self.tiltMotor.setupDefaults()
        self.tiltMotor.findHome()
        self.rollMotor.setupDefaults()
        self.tiltMotor.setupDefaults()
        print("tilt: ", self.tiltMotor.motor.get_position(), "roll: ", self.rollMotor.motor.get_position())
        self.isSetup = True
    
    def getPositionReached(self) -> bool:
        _ret = False
        if ((self.tiltMotor.getPositionSteps() == angleToMicrostep(self.tiltTargetTargetAngle)) and (self.rollMotor.getPositionSteps() == angleToMicrostep(self.rollTargetTargetAngle))):
            print("Position Reached")
            _ret = True
        if((self.tiltMotor.motor.get_current_speed() == 0) and (self.rollMotor.motor.get_current_speed() == 0)):
            print("Not Moving")
            _ret = True
        return _ret

    def waitPositionReached(self):
        while(self.getPositionReached() == False):
            pass
        