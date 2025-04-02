# from globals import *
from pytrinamic.modules import TMCM1110
import math
import time

ROLL_HOME_GPI = 1

TILT_HOME_GPI = 1
TILT_LIMIT_GPI = 2  #Hit Limit Switch (This should be in interrupt)
TILT_OPTICAL_IS_BACK = 0
TILT_OPTICAL_IS_FORWARD = 1

STEP_ANGLE = 1.8
U_STEP = 256
FULL_STEP = (360. / STEP_ANGLE) * U_STEP
MAX_VELOCITY = 1000
MAX_ACCELERATION = 1000

HOME_VELOCITY = 500
HOMING_NUDGE_ANGLE = 10

# This is configured for master (roll motor) -> slave (tilt motor) control, 
# in the attempt to tighten steps between the two meshed motors

class ControllerSet():
    def __init__(self, _rollMotorModule: TMCM1110, _tiltMotorModule: TMCM1110, _armID:int = 0):

        self.rollMotorModule = _rollMotorModule
        self.tiltMotorModule = _tiltMotorModule

        self.rollMotor = self.rollMotorModule.motors[0]
        self.tiltMotor = self.rollMotorModule.motors[1]
        self.tiltMotorController = self.tiltMotorModule.motors[0] #Annoyingly, this class abstracts as axis0 only

        self.armID = _armID
        self.tiltTargetTargetAngle = int(0)
        self.rollTargetTargetAngle = int(0)
        self.sequencePosition = int(0)
        self.isSetup = bool(False)
        self.isHoming = bool(False)

        self.fullTiltAngle = int(80)
        self.fullRollAngle = int(360)
        self.ramp_devisor = int(12)
        self.pulse_devsor = int(4)
        self.max_acceleration = int(1000)


        self.stopMotors()
        self.zeroMotors()

        self.setMotorModuleDefaults(self.tiltMotorController, isSlave=True)
        self.setMotorModuleDefaults(self.rollMotor, isSlave=False)

        self.setControllerAxisRampDefaults(self.tiltMotor, isMaster=False)
        self.setControllerAxisRampDefaults(self.rollMotor, isMaster=True)


        #Master Controller has both end switches, switch the polarity for twinned 2 wire interrupt
        # self.rollMotorModule.set_axis_parameter(ap_type=TMCM1110.GP0.EndSwitchPolarity, value=1)
        # self.disable_limit_switches(_motorModule = self.rollMotorModule, _axis = 1, _value = 0)

    def disable_limit_switches(self, _motorModule:TMCM1110 = None, _axis:int = 0, _value:int = 1):
        _motorModule.set_axis_parameter(axis=_axis, ap_type=TMCM1110._MotorTypeA.AP.LeftLimitSwitchDisable, value=_value)
        _motorModule.set_axis_parameter(axis=_axis, ap_type=TMCM1110._MotorTypeA.AP.RightLimitSwitchDisable, value=_value)

    def setMotorModuleDefaults(self, _motorController:TMCM1110._MotorTypeA = None, isSlave:bool = False):
        _motorController.drive_settings.set_max_current(255)
        _motorController.drive_settings.set_standby_current(255)
        # _motorController.drive_settings.set_boost_current(30)     #Check this 
        _motorController.drive_settings.set_microstep_resolution(TMCM1110._MotorTypeA.ENUM.MicrostepResolution256Microsteps)  #U_STEP n

        # _motorController.stallguard2.set_threshold(20)        #Setting these add the stall guard back in 
        # _motorController.stallguard2.set_stop_velocity(velocity=20)
        _motorController.stallguard2.set_filter(enable_filter=0)

        if(isSlave):
            _motorController.set_axis_parameter(ap_type=TMCM1110._MotorTypeA.AP.StepDirectionMode, value=1)
        
        _motorController.stop()

    def setControllerAxisRampDefaults(self, _motorController:TMCM1110._MotorTypeA = None, isMaster:bool = False):
        #Acelleration   0 -> 2047
        #alower_limit = 2^ramp_div−pulse_div −1
        #aupper_limit = 2^ramp_div−pulse_div+12 −1
        #Velocity       0 -> 2047

        #These are tuned for light velocity/acelleration headroom.
        _motorController.set_axis_parameter(ap_type=TMCM1110._MotorTypeA.AP.RampDivisor, value=12)
        _motorController.set_axis_parameter(ap_type=TMCM1110._MotorTypeA.AP.PulseDivisor, value=4)
        _motorController.set_axis_parameter(ap_type=TMCM1110._MotorTypeA.AP.MaxAcceleration, value=self.max_acceleration)
        

    def angleToMicrostep(self, angle) -> int:
        ret = FULL_STEP/360.
        ret *= angle
        ret = int(ret)
        return ret

    def microstepToAngle(self, microstep) -> float:
        ret = 360./FULL_STEP
        ret *= microstep
        return ret
    
    def get_roll_axis_parameter(self, ap_type:int = None):
        self.rollMotor.get_axis_parameter()


    #This Takes ANGLE in Degrees relative to centre reference (0, 0) 
    # eg. extreme rotate left and back would be: 
    # -90, -90, speed, acceleration
    # ONLY USED ONCE HOMED

    #Annoyingly Axis parmeters not abstracted in motor class for non axis0
    def getActualPosition(self, _axis:int=0):
        return self.rollMotorModule.get_axis_parameter(ap_type=TMCM1110._MotorTypeA.AP.ActualPosition, axis=_axis,signed=True)
    
    def setActualPosition(self, _axis:int=0, _value:int=0):
        return self.rollMotorModule.set_axis_parameter(ap_type=TMCM1110._MotorTypeA.AP.ActualPosition, axis=_axis,signed=True, value=_value)
    
    def setMaxAcceleration(self, _axis:int=0, _value:int=0):
        return self.rollMotorModule.set_axis_parameter(ap_type=TMCM1110._MotorTypeA.AP.MaxAcceleration, axis=_axis,signed=False, value=_value)
    
    def setTargetRotationAngle(self, _rollAngle:float = 0, _tiltAngle:float = 0, _velocity:int = 100, _acceleration:int = 50) -> None:
        
        # Clipping to 'fullRotationAngle' (distance between limit switch left/right)
        tiltCentreAngle = self.fullTiltAngle / 2.
        rollCentreAngle = self.fullRollAngle / 2.
        tiltAngle = max(min(_tiltAngle, tiltCentreAngle), tiltCentreAngle * -1) 
        # rollAngle = max(min(_rollAngle, rollCentreAngle), rollCentreAngle * -1) 

        # _rollAngle = rollCentreAngle + rollAngle 
        # _tiltAngle = tiltCentreAngle + tiltAngle

        _tiltAngle = tiltAngle
        # _rollAngle = rollAngle

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
        mult = 1.
        if(rollStepDistance > tiltStepDistance):
            mult = (tiltStepDistance / rollStepDistance)
            _tiltVelocity = _velocity * mult
            _tiltAcceleration = _acceleration * mult
        
        elif(rollStepDistance < tiltStepDistance):
            mult = (rollStepDistance / tiltStepDistance)
            _rollVelocity = _velocity * mult
            _rollAcceleration = _acceleration * mult

        if(rollStepDistance > 0):
            self.rollMotor.linear_ramp.set_max_acceleration(int(_rollAcceleration))
            self.rollMotor.move_to(position= int(self.angleToMicrostep(_rollAngle)), velocity=int(_rollVelocity))
        
        if(tiltStepDistance > 0):
            self.tiltMotor.linear_ramp.set_max_acceleration(int(_tiltAcceleration))
            self.tiltMotor.move_to(position= int(self.angleToMicrostep(_tiltAngle)), velocity=int(_tiltVelocity))
        

        #Remove This to be NON-blocking for multi arm programming
        # while((self.rollMotor.getIsPositionReached() == False) or (self.tiltMotor.getIsPositionReached() == False)):
        #     pass

  #  # Example usage:
  #  current_position = 0
  #  target_position = 10000
  #  max_acceleration = 5000  # steps/s^2
  #  max_velocity = 2000  # steps/s
  #  ramp_divisor = 2
  #  pulse_divisor = 1
#
  #  time_required = calculate_ramp_time(current_position, target_position, max_acceleration, max_velocity, ramp_divisor, pulse_divisor)
  #  print(f"Total ramp time: {time_required:.2f} seconds")
    
    def stopMotors(self):
        self.rollMotor.stop()
        self.tiltMotor.stop()
    
    def zeroMotors(self):
        self.rollMotor.set_actual_position(position=0)
        self.tiltMotor.set_actual_position(position=0)

    def set_target_position(self, _angle):
        #0: position mode. Steps are generated, when
        #the parameters actual position and target
        #position differ. Trapezoidal speed ramps are
        #provided.```````````````````
        # -*+9`
        self.rollMotor.set_axis_parameter(ap_type=TMCM1110._MotorTypeA.AP.RampType, value=1)
        self.tiltMotor.set_axis_parameter(ap_type=TMCM1110._MotorTypeA.AP.RampType, value=1)

        self.rollMotor.set_target_position(position=int(self.angleToMicrostep(_angle)))
        self.tiltMotor.set_target_position(position=int(self.angleToMicrostep(_angle)))
    
    def rollDisc(self, _angle, _velocity, _accelleration=MAX_ACCELERATION):
        self.rollMotor.set_axis_parameter(ap_type=TMCM1110._MotorTypeA.AP.MaxAcceleration, value=_accelleration)
        self.tiltMotor.set_axis_parameter(ap_type=TMCM1110._MotorTypeA.AP.MaxAcceleration, value=_accelleration)
        
        self.tiltMotor.move_to(position=int(self.angleToMicrostep(_angle)), velocity=int(_velocity))
        self.rollMotor.move_to(position=int(self.angleToMicrostep(_angle)), velocity=int(_velocity))
    
    def homeMotors(self):
        self.stopMotors()
        self.isHoming = True
        self.rollDisc(_angle=-360, _velocity=HOME_VELOCITY, _accelleration=MAX_ACCELERATION)


        while(self.rollMotorModule.get_digital_input(ROLL_HOME_GPI) == 1):
            pass

        self.stopMotors()
        self.rollMotor.set_actual_position(position=0)
        print("Found Roll Home, set as Zero")

        #Master Controller has both end switches, switch the polarity for twinned 2 wire interrupt
        self.rollMotorModule.set_global_parameter(gp_type=TMCM1110.GP0.EndSwitchPolarity, bank=0, value=1)
        self.disable_limit_switches(_motorModule = self.rollMotorModule, _axis = 1, _value=0)
        
        
        homingDirection = 1
        homingAttempts = 0
        nudgeStep = self.angleToMicrostep(15)
        self.tiltMotor.set_actual_position(position=0)
        homingPath = self.angleToMicrostep(360)

        while(self.isHoming == True):
            homingPath *= homingDirection
            
            self.tiltMotor.move_to(position=homingPath, velocity=HOME_VELOCITY)
            start_position = self.tiltMotor.get_actual_position()

            while(self.getIsMoving()):
                if(self.tiltMotorModule.get_digital_input(TILT_HOME_GPI) == 1):
                    self.tiltMotor.stop()
                    self.tiltMotor.set_actual_position(0)
                    self.isHoming = False
                    print("Found Home..: ")
                    break
                
                elif((self.tiltMotorModule.get_digital_input(TILT_LIMIT_GPI) == 0) and 
                     (abs(start_position - self.tiltMotor.get_actual_position()) > nudgeStep)):
                    
                    self.tiltMotor.stop()
                    homingDirection *= -1    #Youve Hit Limit Go Backwards
                    nudgeStep = 10
                    print("Tilt Hit Limit, New Path: ", homingDirection)
                    homingAttempts += 1
                    break

        self.isHoming = False
    
    def homeMotors2(self):
        self.stopMotors()
        self.isHoming = True

        self.disable_limit_switches(self.rollMotorModule, _axis = 1, _value = 1) 

        ##Home Roll
        self.rollDisc(_angle=-360, _velocity=HOME_VELOCITY, _accelleration=MAX_ACCELERATION)
        while(self.rollMotorModule.get_digital_input(ROLL_HOME_GPI) == 1):
            pass
        self.stopMotors()
        self.rollMotor.set_actual_position(position=0)
        print("Found Roll Home, set as Zero")
        ##########################################################################################

        ##Home Tilt
        #Master Controller has both end switches, switch the polarity for twinned 2 wire interrupt
        self.tiltMotor.set_actual_position(0)
        
        self.rollMotorModule.set_global_parameter(gp_type=TMCM1110.GP0.EndSwitchPolarity, bank=0, value=1)
        self.disable_limit_switches(_motorModule = self.rollMotorModule, _axis = 1, _value=0)
        
        self.tiltMotor.move_to(int(-FULL_STEP), velocity=HOME_VELOCITY)
        while(self.getIsMoving() == True):
            pass
        self.tiltMotor.stop()
        self.tiltMotor.set_actual_position(0)
        
        #Nudge Forward with limit switch disable
        self.disable_limit_switches(self.rollMotorModule, _axis = 1, _value = 1) 
        self.tiltMotor.move_to(self.angleToMicrostep(HOMING_NUDGE_ANGLE), velocity=HOME_VELOCITY)
        while(self.getIsMoving() == True):
            pass
        self.tiltMotor.stop()
        self.disable_limit_switches(self.rollMotorModule, _axis = 1, _value = 0) 
        
        #Move to Other Extreme
        self.tiltMotor.move_to(int(FULL_STEP), velocity=HOME_VELOCITY)
        while(self.getIsMoving() == True):
            pass
        self.tiltMotor.stop()
        self.fullTiltAngle = self.microstepToAngle(self.tiltMotor.get_actual_position())

        self.disable_limit_switches(self.rollMotorModule, _axis = 1, _value = 1) 
        self.tiltMotor.move_to(int(self.angleToMicrostep(self.fullTiltAngle/2)), velocity=HOME_VELOCITY)
        while(self.getIsMoving() == True):
            pass
        self.tiltMotor.stop()


        print("Found Tilt Home: ", self.microstepToAngle(self.tiltMotor.get_actual_position()),  "deg set as Zero, re-enabling limit switches")
        self.tiltMotor.set_actual_position(0)
        self.disable_limit_switches(self.rollMotorModule, _axis = 1, _value = 0) 

        ##########################################################################################

    def homeMotors3(self):
        self.isHoming = True
        self.stopMotors()
        self.rollMotor.set_actual_position(position=0)
        self.tiltMotor.set_actual_position(position=0)

        self.disable_limit_switches(self.rollMotorModule, _axis = 1, _value = 1) 
        self.rollMotorModule.set_global_parameter(gp_type=TMCM1110.GP0.EndSwitchPolarity, bank=0, value=1)

        ##Home Roll
        # self.rollDisc(_angle=-360, _velocity=HOME_VELOCITY, _accelleration=MAX_ACCELERATION)
        while(self.rollMotorModule.get_digital_input(ROLL_HOME_GPI) == 1):
            self.rollMotor.move_by(int(U_STEP*-2))
            self.tiltMotor.move_by(int(U_STEP*-2))
            # self.rollMotor.coolstep.calibrate()
            # while(self.getIsMoving()):
            #     pass
        
        self.stopMotors()
        self.rollMotor.set_actual_position(position=0)
        print("Found Roll Home, set as Zero")
        ##########################################################################################

        ##Home Tilt
        #Master Controller has both end switches, switch the polarity for twinned 2 wire interrupt
        self.tiltMotor.set_actual_position(0)
        # homePath = self.angleToMicrostep(90)
        direction= 1
        path = int(U_STEP)
        foundTiltHome = False
        currentOrientation = self.tiltMotorModule.get_digital_input(TILT_HOME_GPI)
        
        if(currentOrientation == TILT_OPTICAL_IS_FORWARD):  #Go backwards if tilt is forward 
            direction *= -1

        # self.tiltMotor.move_to(position=int(homePath), velocity=int(HOME_VELOCITY/2))
        while(currentOrientation == self.tiltMotorModule.get_digital_input(TILT_HOME_GPI)):
            self.tiltMotor.move_by(path * direction)
            # self.tiltMotorModule.motors[0].coolstep.calibrate()

        self.tiltMotor.stop()
        foundTiltHome = True
        
        if(foundTiltHome):
            self.tiltMotor.set_actual_position(0)
            print("Found Home")
        else:
            print("Hit Limit Stop, when trying to home Tilt")

        ##########################################################################################

    # def linearHomingUpdatotr(self) -> int:
    #     if(self.isHoming):
    #         if
    #     return int(1)
        
    def getPositionReached(self) -> bool:
        _ret = False
        if((self.tiltMotor.get_position_reached()) and (self.rollMotor.get_position_reached())):
            print("Position Reached")
            _ret = True
        elif(self.getIsMoving() == False):
            print("Not Moving")
            _ret = True
        return _ret

    def hasHitLimits(self) -> bool:
        return self.tiltMotorModule.get_digital_input(port=TILT_LIMIT_GPI)

    def getIsMoving(self) -> bool:
        if((self.tiltMotor.get_actual_velocity() == 0) and (self.rollMotor.get_actual_velocity() == 0)):
            print("NOT MOVING")
            return False
        else:
            return True

    def waitPositionReached(self):
        while(self.getPositionReached() == False):
            # print("self.tiltMotor.get_actual_position()", self.microstepToAngle(self.tiltMotor.get_actual_position()) - self.microstepToAngle(self.rollMotor.get_actual_position()))
            # print("self.rollMotor.get_actual_position()", self.microstepToAngle(self.rollMotor.get_actual_position()))
            pass







# from multiprocessing import shared_memory, Process
# import numpy as np

# def worker(shared_mem_name, shape, dtype):
#     # Attach to existing shared memory
#     existing_shm = shared_memory.SharedMemory(name=shared_mem_name)
#     array = np.ndarray(shape, dtype=dtype, buffer=existing_shm.buf)
    
#     # Modify shared memory (example: multiply by 2)
#     array *= 2

#     # Close the shared memory (but do not unlink)
#     existing_shm.close()

# if __name__ == "__main__":
#     # Create a NumPy array and shared memory
#     shape = (5,)
#     dtype = np.int64
#     data = np.array([1, 2, 3, 4, 5], dtype=dtype)

#     shm = shared_memory.SharedMemory(create=True, size=data.nbytes)
#     shared_array = np.ndarray(shape, dtype=dtype, buffer=shm.buf)
#     shared_array[:] = data[:]  # Copy data to shared memory

#     # Create and start worker process
#     p = Process(target=worker, args=(shm.name, shape, dtype))
#     p.start()
#     p.join()

#     # Read updated data from shared memory
#     print("Updated array:", shared_array)

#     # Cleanup
#     shm.close()
#     shm.unlink()  # Unlink after all processes are done