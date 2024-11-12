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

    #Get Max Speed PP/s to make overall roattion time
    def setTargetRotationAngle(self, rollAngle:float = 0, tiltAngle:float = 0, speed:int = 100, velocity:int = 50):
        #in here need current angle to compare against target angle
        tiltCentreAngle = self.tiltMotor.fullRotationAngle / 2.
        rollCentreAngle = self.rollMotor.fullRotationAngle / 2.
        
        rollAngle = max(min(rollAngle, rollCentreAngle), rollCentreAngle * -1)
        tiltAngle = max(min(tiltAngle, tiltCentreAngle), tiltCentreAngle * -1)

        _rollAngle = rollCentreAngle + rollAngle 
        _tiltAngle = tiltCentreAngle + tiltAngle

        _tiltAngle += _rollAngle     #roll is Master, and locked on axis

        _tiltSpeed = speed
        _rollSpeed = speed
        _rollVelocity = velocity
        _tiltVelocity = velocity
        mult = 1

        currentTiltStep = self.tiltMotor.getPositionSteps()
        currentRollStep = self.rollMotor.getPositionSteps()

        tiltDistance = abs(angleToMicrostep(_tiltAngle) - currentTiltStep)
        rollDistance = abs(angleToMicrostep(_rollAngle) - currentRollStep)

        #Match Up speeds
        if(rollDistance > tiltDistance):
            mult = (tiltDistance / rollDistance)
            _tiltSpeed = speed * mult
            # _tiltVelocity = velocity * mult
        elif(rollDistance < tiltDistance):
            mult = (rollDistance / tiltDistance)
            _rollSpeed = speed * mult
            # _rollVelocity = velocity * mult


        self.rollMotor.setMotorTaget(int(_rollAngle), _rollSpeed, _rollVelocity)
        self.tiltMotor.setMotorTaget(int(_tiltAngle), _tiltSpeed, _tiltVelocity)

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