from globals import *
from pylablib.devices import Trinamic
import time


class MotorController():
    def __init__(self, comport):
        self.fullRotationSteps = 0
        self.fullRotationAngle = 0
        self.motor = Trinamic.TMCM1110(comport)
        self.stallguardThreshold = 20
        self.name = ""
        self.axisMult = 1
        self.targetAngle = 0.0
        self.armID=0
        # self.setupDefaults()

    def setupDefaults(self):
        self.motor.set_microstep_resolution(U_STEP)
        self.motor.set_axis_parameter(parameter=4, value=MAX_SPEED)
        self.motor.set_axis_parameter(parameter=5, value=MAX_ACCELERATION)
        self.motor.set_axis_parameter(parameter=6, value=100) #max current
        self.motor.set_axis_parameter(parameter=7, value=70) #standby current
        self.motor.set_axis_parameter(axis=0, parameter=173, value=1)   #stallGuard Filter
        self.motor.set_axis_parameter(axis=0, parameter=174, value=self.stallguardThreshold)   #stallGuard threshold
        self.motor.set_axis_parameter(axis=0, parameter=181, value=0)   #stallGuard stop velocity
        self.motor.setup_limit_switches(axis=0, left_enable=False, right_enable=False)
        self.motor.set_position_reference(pos= 0, axis=0)
        self.motor.stop()

    def findBounds(self):
        self.motor.move_to(-FULL_STEP, axis=0)
        while(self.motor.get_current_speed(axis=0) != 0):
            pass
        self.motor.stop()
        
        self.motor.set_position_reference(axis=0, pos=0)

        self.motor.move_to(FULL_STEP, axis=0)
        while(self.motor.get_current_speed(axis=0) != 0):
            pass
        print("stop")
        self.motor.stop()
        
        self.fullRotationSteps = self.motor.get_position(axis=0)
        self.fullRotationAngle = microstepToAngle(self.fullRotationSteps)
        self.motor.move_to(0, axis=0)
        
        while(self.motor.get_current_speed(axis=0) != 0):
            pass

        print("Motor: ", self.name, "Position: ", self.motor.get_position(), "Angle: ", microstepToAngle(self.motor.get_position()))
        print("Motor: ", self.name, "FullRotationAngle: ", self.fullRotationAngle, "FullRotationSteps: ", self.fullRotationSteps)
    
    def findHome(self, gpi_pin:int=0):
        self.motor.stop()
        self.motor.move_to(-FULL_STEP, axis=0)
        while(self.motor.get_current_speed(axis=0) != 0):
            if(self.getGPI(gpi_pin) == 1):
                self.motor.stop()
        # self.motor.stop()
        print(self.name + "Found Home, setting as Zero")
        self.motor.set_position_reference(axis=0, pos=0)
        return 1
    
    def lockMotor(self):
        self.motor.set_axis_parameter(parameter=7, value=128) #standby current
        self.motor.stop()
    
    def moveto(self, ustep:int=0):
        self.motor.move_to(ustep, axis=0)
    
    def stop(self):
        self.motor.stop()
    
    def freeMotor(self):
        self.motor.set_axis_parameter(parameter=7, value=0) #standby current
        self.motor.stop()
    
    def getControllerPrimaryAddress(self) -> int:
        self.armID = self.motor.get_global_parameter(bank=0, addr=0, parameter=66)
        return self.armID
    
    def getControllerSecondaryAddress(self) -> int:
        return self.motor.get_global_parameter(bank=0, addr=0, parameter=87)

    def getIsPositionReached(self) -> bool:
        ret = False
        if(self.getPositionSteps() == angleToMicrostep(self.targetAngle)):
            ret = True
        return ret
    
    def getIsMoving(self) -> bool:
        ret = True
        if(self.motor.get_current_speed() == int(0)):
            ret = False
        return ret

    def getPositionSteps(self) -> int:
        return self.motor.get_position()

    def getPositionAngle(self) -> float:
        return microstepToAngle(self.motor.get_position())
    
    def set_limit_switches(self, isLimiting:bool = True):
        self.motor.setup_limit_switches(axis=0, left_enable=isLimiting, right_enable=isLimiting)
    
    def setMotorTaget(self, angle = 0, speed:int = MAX_SPEED, acceleration:int = MAX_ACCELERATION):
        self.targetAngle = angle
        self.motor.set_axis_parameter(parameter=4, value=speed)
        self.motor.set_axis_parameter(parameter=5, value=acceleration)
        self.motor.move_to(angleToMicrostep(self.targetAngle))
        print("ArmNumber", self.armID, "Motor", self.name, "MicroStep: ", angleToMicrostep(self.targetAngle), "Angle: ", angle, "Speed: ", speed, "Velocity: ", acceleration)

    def getGPI(self, port) -> int:
        reply = self.motor.get_general_input(bank=0, port=port)
        if(reply):
            return reply.value
        
    
    def getHomingSwitch(self, port)-> int:
        reply = self.motor.get_general_input(bank=1, port=port)
        if(reply):
            return reply.value
    
        
    
    

        