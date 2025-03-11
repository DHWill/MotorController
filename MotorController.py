from globals import *
import time

from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1110


class MotorController():
    def __init__(self, interface:ConnectionManager, motorID:int):
        self.module = TMCM1110(interface, module_id=motorID)
        self.motor = self.module.motors[0]
        self.fullRotationSteps = 0
        self.fullRotationAngle = 0
        self.stallguardThreshold = 20
        self.name = ""
        self.axisMult = 1
        self.targetAngle = 0.0
        self.armID=0
        # self.setupDefaults()

    def setupDefaults(self):
        self.motor.drive_settings.set_max_current(100)
        self.motor.drive_settings.set_standby_current(70)
        self.motor.drive_settings.set_boost_current(30)     #Check this 
        self.motor.drive_settings.set_microstep_resolution(self.motor.ENUM.MicrostepResolution256Microsteps)
        
        self.motor.linear_ramp.set_max_acceleration(MAX_ACCELERATION)
        self.motor.linear_ramp.set_max_velocity(MAX_VELOCITY)

        self.motor.stallguard2.set_filter(enable_filter=0)
        self.motor.stallguard2.set_threshold(self.stallguardThreshold)
        self.motor.stallguard2.set_stop_velocity(velocity=2)
        self.motor.set_actual_position(0)
        self.motor.stop()

    def findBounds(self):
        self.motor.move_to(-FULL_STEP, axis=0)
        while(self.motor.get_actual_velocity() != 0):
            pass
        self.motor.stop()
        
        self.motor.set_actual_position(axis=0, pos=0)

        self.motor.move_to(FULL_STEP, axis=0)
        while(self.motor.get_actual_velocity() != 0):
            pass
        print("stop")
        self.motor.stop()
        
        self.fullRotationSteps = self.motor.get_actual_position()
        self.fullRotationAngle = microstepToAngle(self.fullRotationSteps)
        self.motor.move_to(0, axis=0)
        
        while(self.motor.get_actual_velocity() != 0):
            pass

        print("Motor: ", self.name, "Position: ", self.motor.get_actual_position(), "Angle: ", microstepToAngle(self.motor.get_actual_position()))
        print("Motor: ", self.name, "FullRotationAngle: ", self.fullRotationAngle, "FullRotationSteps: ", self.fullRotationSteps)
    
    def findHome(self, gpi_pin:int=0):
        self.motor.stop()
        self.motor.move_to(-FULL_STEP, axis=0)
        while(self.motor.get_actual_velocity(axis=0) != 0):
            if(self.getGPI(gpi_pin) == 1):
                self.motor.stop()
        # self.motor.stop()
        print(self.name + "Found Home, setting as Zero")
        self.motor.set_actual_position(position=0)
        return 1
    
    def lockMotor(self):
        self.motor.drive_settings.set_standby_current(70)
        self.motor.stop()
    
    def moveto(self, ustep:int=0):
        self.motor.move_to(position= ustep, velocity=self.motor.linear_ramp.get_max_velocity())
    
    def stop(self):
        self.motor.stop()
    
    def freeMotor(self):
        self.motor.drive_settings.set_standby_current(0)
        self.motor.stop()
    
    def getControllerSerialAddress(self) -> int:
        # self.armID = self.motor.get_global_parameter(bank=0, addr=0, parameter=66)
        self.armID = self.module.get_global_parameter(gp_type=self.module.GP0.SerialAddress, bank=0, signed=False)
        return self.armID
    
    def getControllerSecondarySerialAddress(self) -> int:
        return self.module.get_global_parameter(gp_type=self.module.GP0.serialSecondaryAddress, bank=0, signed=False)

    def getIsPositionReached(self) -> bool:
        return self.motor.get_position_reached()
    
    def getIsMoving(self) -> bool:
        ret = True
        if(self.motor.get_actual_velocity() == int(0)):
            ret = False
        return ret

    def getPositionSteps(self) -> int:
        return self.motor.get_actual_position()

    def getPositionAngle(self) -> float:
        return microstepToAngle(self.motor.get_actual_position())
    
    def set_limit_switches(self, isLimiting:bool = True):
        self.motor.setup_limit_switches(axis=0, left_enable=isLimiting, right_enable=isLimiting)        #TO FIX
    
    def setMotorTaget(self, angle = 0, _velocity:int = MAX_VELOCITY, _acceleration:int = MAX_ACCELERATION):
        self.targetAngle = angle
        # self.motor.set_axis_parameter(parameter=4, value=speed)
        # self.motor.set_axis_parameter(parameter=5, value=acceleration)
        self.motor.move_to(position=angleToMicrostep(self.targetAngle), velocity=_velocity)
        print("ArmNumber", self.armID, "Motor", self.name, "MicroStep: ", angleToMicrostep(self.targetAngle), "Angle: ", angle, "Veloctiy: ", _velocity, "Acelleration: ", _acceleration)

    def getGPI(self, port) -> int:
        reply = self.module.get_digital_input(x=port)
        # if(reply):
        return reply
        
    def getAnalog(self, port) -> int:
        reply = self.module.get_analog_input(x=port)
        # if(reply):
        return reply
        

    
        
    
    

        