# from globals import *
from MotorController import MotorController
from pytrinamic.modules import TMCM1110

ROLL_HOME_GPI = 1
TILT_HOME_GPI = 1
TILT_LIMIT_GPI = 2  #Hit Limit Switch (This should be in interrupt)
STEP_ANGLE = 1.8
U_STEP = 256
FULL_STEP = (360. / STEP_ANGLE) * U_STEP
MAX_VELOCITY = 200
MAX_ACCELERATION = 50

class ControllerSet():
    def __init__(self, _rollMotorController: TMCM1110, _tiltMotorController: TMCM1110, _armID:int = 0):
        self.rollMotorController = _rollMotorController
        self.tiltMotorController = _tiltMotorController
        self.rollMotor = self.rollMotorController.motors[0]
        self.tiltMotor = self.tiltMotorController.motors[0]
        self.armID = _armID
        self.tiltTargetTargetAngle = 0
        self.rollTargetTargetAngle = 0
        self.sequencePosition = 0
        self.isSetup = False
        self.isHoming = False
        self.fullTiltAngle = 80
        self.fullRollAngle = 360
    
    def setMotorDefault(self, _motor:TMCM1110._MotorTypeA = None):
        _motor.set_actual_position(0)
        _motor.drive_settings.set_max_current(100)
        _motor.drive_settings.set_standby_current(70)
        _motor.drive_settings.set_boost_current(30)     #Check this 
        _motor.drive_settings.set_microstep_resolution(TMCM1110._MotorTypeA.ENUM.MicrostepResolution256Microsteps)  #U_STEP not

        _motor.linear_ramp.set_max_acceleration(MAX_ACCELERATION)
        _motor.linear_ramp.set_max_velocity(MAX_VELOCITY)
        # _motor.linear_ramp.set_ramp_enabled()     #CheckThis

        _motor.stallguard2.set_filter(enable_filter=1)
        _motor.stallguard2.set_threshold(8)
        _motor.stallguard2.set_stop_velocity(velocity=2)

        
        _motor.stop()

    def angleToMicrostep(self, angle) -> int:
        ret = FULL_STEP/360.
        ret *= angle
        ret = int(ret)
        return ret

    def microstepToAngle(self, microstep) -> float:
        ret = 360./FULL_STEP
        ret *= microstep
        return ret


    #This Takes ANGLE in Degrees relative to centre reference (0, 0) 
    # eg. extreme rotate left and back would be: 
    # -90, -90, speed, acceleration
    # ONLY USED ONCE HOMED
    def setTargetRotationAngle(self, _rollAngle:float = 0, _tiltAngle:float = 0, _velocity:int = 100, _acceleration:int = 50):
        
        # Clipping to 'fullRotationAngle' (distance between limit switch left/right)
        tiltCentreAngle = self.fullTiltAngle / 2.
        rollCentreAngle = self.fullRollAngle / 2.
        tiltAngle = max(min(_tiltAngle, tiltCentreAngle), tiltCentreAngle * -1) 
        rollAngle = max(min(_rollAngle, rollCentreAngle), rollCentreAngle * -1) 

        # _rollAngle = rollCentreAngle + rollAngle 
        # _tiltAngle = tiltCentreAngle + tiltAngle
        _tiltAngle = tiltAngle
        _rollAngle = rollAngle

        _tiltAngle += _rollAngle     #roll is Master, and locked on axis

        _tiltVelocity = _velocity
        _rollVelocity = _velocity

        _rollAcceleration = _acceleration
        _tiltAcceleration = _acceleration

        currentTiltStep = self.tiltMotor.get_actual_position()
        currentRollStep = self.rollMotor.get_actual_position()

        tiltStepDistance = abs(self.angleToMicrostep(_tiltAngle) - currentTiltStep)
        rollStepDistance = abs(self.angleToMicrostep(_rollAngle) - currentRollStep)

        #Match Up speeds
        mult = 1
        if(rollStepDistance > tiltStepDistance):
            mult = (tiltStepDistance / rollStepDistance)
            _tiltVelocity = _velocity * mult
            _tiltAcceleration = _acceleration * mult
        
        elif(rollStepDistance < tiltStepDistance):
            mult = (rollStepDistance / tiltStepDistance)
            _rollVelocity = _velocity * mult
            _rollAcceleration = _acceleration * mult


        self.rollMotor.move_to(position= int(self.angleToMicrostep(_rollAngle)), velocity=int(_rollVelocity))
        self.tiltMotor.move_to(position= int(self.angleToMicrostep(_tiltAngle)), velocity=int(_tiltVelocity))

        #Remove This to be NON-blocking for multi arm programming
        # while((self.rollMotor.getIsPositionReached() == False) or (self.tiltMotor.getIsPositionReached() == False)):
        #     pass


    def setArmLimitSwitches(self, _isLimiting:bool = False):
        pass
    
    def stopMotors(self):
        self.rollMotor.stop()
        self.tiltMotor.stop()
    
    def zeroMotors(self):
        self.rollMotor.set_actual_position(position=0)
        self.tiltMotor.set_actual_position(position=0)
    
    def rollDisc(self, _angle, _velocity):
        self.rollMotor.move_to(position=int(self.angleToMicrostep(_angle)), velocity=int(_velocity))
        self.tiltMotor.move_to(position=int(self.angleToMicrostep(_angle)), velocity=int(_velocity))
    
    def homeMotors(self):
        self.isHoming = True
        self.rollMotor.stop()
        self.tiltMotor.stop()

        self.setMotorDefault(self.rollMotor)
        self.setMotorDefault(self.tiltMotor)

        self.rollDisc(_angle=-360, _velocity=100)


        while(self.rollMotorController.get_digital_input(ROLL_HOME_GPI) == 0):
            pass

        self.stopMotors()
        self.rollMotor.set_actual_position(position=0)
        print("Found Roll Home, set as Zero")
        
        
        homingDirection = 1
        homingAttempts = 0
        nudgeStep = self.angleToMicrostep(10)
        self.tiltMotor.set_actual_position(position=0)
        homingPath = self.angleToMicrostep(360)

        while(self.isHoming == True):
            homingPath *= homingDirection
            
            self.tiltMotor.move_to(position=homingPath, velocity=100)
            start_position = self.tiltMotor.get_actual_position()

            while(self.getIsMoving()):
                if(self.tiltMotorController.get_digital_input(TILT_HOME_GPI) == 1):
                    self.tiltMotor.stop()
                    self.tiltMotor.set_actual_position(0)
                    self.isHoming = False
                    print("Found Home..: ")
                    break
                
                elif((self.tiltMotorController.get_digital_input(TILT_LIMIT_GPI) == 0) and 
                     (abs(start_position - self.tiltMotor.get_actual_position()) > nudgeStep)):
                    
                    self.tiltMotor.stop()
                    homingDirection *= -1    #Youve Hit Limit Go Backwards
                    nudgeStep = 10
                    print("Tilt Hit Limit, New Path: ", homingDirection)
                    homingAttempts += 1
                    break

        self.isHoming = False

    
    def getPositionReached(self) -> bool:
        _ret = False
        if((self.tiltMotor.get_position_reached()) and (self.rollMotor.get_position_reached())):
            print("Position Reached")
            _ret = True
        elif(self.getIsMoving() == False):
            print("Not Moving")
            _ret = True
        return _ret

    def hasHitLimits(self):
        return self.tiltMotor.getGPI(port=TILT_LIMIT_GPI)

    def getIsMoving(self) -> bool:
        if((self.tiltMotor.get_actual_velocity() == 0) and (self.rollMotor.get_actual_velocity() == 0)):
            print("NOT MOVING")
            return False
        else:
            return True

    def waitPositionReached(self):
        while(self.getPositionReached() == False):
            pass

