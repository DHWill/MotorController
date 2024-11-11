from pylablib.devices import Trinamic
import serial.tools.list_ports
import ctypes
from ctypes import wintypes
import time
import random as rand
import math

STEP_ANGLE = 1.8
U_STEP = 256
FULL_STEP = (360. / STEP_ANGLE) * U_STEP


MAX_SPEED = 200
MAX_ACCELERATION = 50

def get_usb_com_ports_with_serial_numbers():
    ports = serial.tools.list_ports.comports()
    usb_ports_with_serial = []

    for port in ports:
        if "USB" in port.description:
            # Parse the serial number from the hwid if available
            hwid_parts = port.hwid.split(" ")
            serial_number = None
            for part in hwid_parts:
                if "SER=" in part:
                    serial_number = part.split("SER=")[-1]
                    break
            
            usb_ports_with_serial.append((port.device, serial_number if serial_number else "N/A"))

    return usb_ports_with_serial



def angleToMicrostep(angle):
    ret = FULL_STEP/360.
    ret *= angle
    ret = int(ret)
    return ret

def microstepToAngle(microstep):
    ret = 360./FULL_STEP
    ret *= microstep
    return ret



# stage1 = Trinamic.TMCM1110("COM4")
# stage2 = Trinamic.TMCMx110("COM4")

class MotorController():
    def __init__(self, comport):
        self.fullRotationSteps = 0
        self.fullRotationAngle = 0
        self.motor = Trinamic.TMCM1110(comport)
        self.stallguardThreshold = 20
        self.name = ""
        self.axisMult = 1
        # self.setupDefaults()

    def setupDefaults(self):
        self.motor.set_microstep_resolution(U_STEP)
        self.motor.set_axis_parameter(parameter=4, value=MAX_SPEED)
        self.motor.set_axis_parameter(parameter=5, value=MAX_ACCELERATION)
        self.motor.set_axis_parameter(parameter=6, value=255) #max current
        self.motor.set_axis_parameter(parameter=7, value=8) #standby current
        self.motor.set_axis_parameter(axis=0, parameter=173, value=1)   #stallGuard Filter
        self.motor.set_axis_parameter(axis=0, parameter=174, value=self.stallguardThreshold)   #stallGuard threshold
        self.motor.set_axis_parameter(axis=0, parameter=181, value=0)   #stallGuard stop velocity
        self.motor.setup_limit_switches(axis=0, left_enable=True, right_enable=True)
        self.motor.stop()

    def findHome(self):
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
        time.sleep(1)
        self.motor.move_to(0, axis=0)
        self.motor.wait_move()

        print("Motor: ", self.name, "Position: ", self.motor.get_position(), "Angle: ", microstepToAngle(self.motor.get_position()))
        print("Motor: ", self.name, "FullRotationAngle: ", self.fullRotationAngle, "FullRotationSteps: ", self.fullRotationSteps)

    
    def lockMotor(self):
        self.motor.set_axis_parameter(parameter=7, value=128) #standby current
        self.motor.stop()
    
    def freeMotor(self):
        self.motor.set_axis_parameter(parameter=7, value=0) #standby current
        self.motor.stop()
    
    def getMotorID(self):
        return self.motor.get_global_parameter(bank=0, addr=0, parameter=66)


#Roll = Serial Adress 0
#Tilt = Serial Adress 1

comportList = get_usb_com_ports_with_serial_numbers()
tilt = None
roll = None
for comport, serial_number in comportList:
    motor = None
    try:
        motor = MotorController(comport)
        motorID = motor.getMotorID()

        if(motorID == 0):
            motor.name = "roll"
            roll = motor
        elif(motorID == 1):
            motor.name = "tilt"
            tilt = motor
    except:
        print("No Motor detected on:", comport)

if((tilt == None) or (roll == None)):
    print("No Motors detected, exiting...")
    quit()



def setMotorTaget(_motor: MotorController = None, angle = "", speed:int = MAX_SPEED, acceleration:int = MAX_ACCELERATION):
    _motor.motor.set_axis_parameter(parameter=4, value=speed)

    _motor.motor.set_axis_parameter(parameter=5, value=acceleration)
    _motor.motor.move_to(angleToMicrostep(angle))
    print("Motor", _motor.name, "MicroStep: ", angleToMicrostep(angle), "Angle: ", angle, "Speed: ", speed, "Velocity: ", acceleration)

#Get Max Speed PP/s to make overall roattion time
def setTargetRotationAngle( _rollMotor: MotorController, _tiltMotor: MotorController, rollAngle:float = 0, tiltAngle:float = 0, speed:int = 100, velocity:int = 50):
    #in here need current angle to compare against target angle

    _rollAngle = rollAngle
    _tiltAngle = tiltAngle + _rollAngle     #roll is Master, and locked on axis

    _tiltSpeed = speed
    _rollSpeed = speed
    _rollVelocity = velocity
    _tiltVelocity = velocity
    mult = 1

    currentTiltStep = _tiltMotor.motor.get_position()
    currentRollStep = _rollMotor.motor.get_position()
    
    tiltDistance = abs(angleToMicrostep(_tiltAngle)- currentTiltStep)
    rollDistance = abs(angleToMicrostep(_rollAngle)- currentRollStep)

    #Match Up speeds
    if(rollDistance > tiltDistance):
        mult = (tiltDistance / rollDistance)
        _tiltSpeed = speed * mult
        # _tiltVelocity = velocity * mult
    elif(rollDistance < tiltDistance):
        mult = (rollDistance / tiltDistance)
        _rollSpeed = speed * mult
        # _rollVelocity = velocity * mult
    
    
    setMotorTaget(_rollMotor, int(_rollAngle), _rollSpeed, _rollVelocity)
    setMotorTaget(_tiltMotor, int(_tiltAngle), _tiltSpeed, _tiltVelocity)
    atPosition = False
    while(atPosition == False):
        if((_tiltMotor.motor.get_current_speed() == 0) or (_rollMotor.motor.get_current_speed() == 0)):
            atPosition = True
            print("Not Moving")
            pass
        if((_tiltMotor.motor.get_position() == angleToMicrostep(_tiltAngle)) and (_rollMotor.motor.get_position() == angleToMicrostep(_rollAngle))):
            atPosition = True
            print("Position Reached")
        pass

    print("tilt: ", tilt.motor.get_position(), "roll: ", roll.motor.get_position())
    print("positionReached")
    return


def setupRoutine( _rollMotor: MotorController, _tiltMotor: MotorController):
    _rollMotor.setupDefaults()
    _tiltMotor.setupDefaults()
    _tiltMotor.freeMotor()  
    _rollMotor.findHome()
    _rollMotor.lockMotor()      
    _tiltMotor.setupDefaults()
    _tiltMotor.findHome()
    _rollMotor.setupDefaults()
    _tiltMotor.setupDefaults()
    print("tilt: ", _tiltMotor.motor.get_position(), "roll: ", _rollMotor.motor.get_position())


def zeroPositionReference():
    roll.motor.set_position_reference(axis=0, pos=0)
    tilt.motor.set_position_reference(axis=0, pos=0)

#Roll Motor Is Master Axes
#Tilt Angle = Roll + Tilt Angle

setupRoutine(roll, tilt)
roll.motor.setup_limit_switches(axis=0, left_enable=False, right_enable=False)
tilt.motor.setup_limit_switches(axis=0, left_enable=False, right_enable=False)


#Limit to +/- 45Deg
tiltCentreAngle = tilt.fullRotationAngle / 2.

#Centre Point 
setTargetRotationAngle(roll, tilt, rollAngle=roll.fullRotationAngle/2, tiltAngle=tiltCentreAngle, speed=200, velocity=50)

while(True):

    print("------------------------------------------------")
    print("Look Bottom Right")
    setTargetRotationAngle(roll, tilt, rollAngle=roll.fullRotationAngle, tiltAngle=tiltCentreAngle-45, speed=200, velocity=20)
    print("------------------------------------------------")
    print("Look Top Left")
    setTargetRotationAngle(roll, tilt, rollAngle=0, tiltAngle=tiltCentreAngle+45, speed=200, velocity=20)
    print("------------------------------------------------")
    print("Look Centre")
    setTargetRotationAngle(roll, tilt, rollAngle=roll.fullRotationAngle/2, tiltAngle=tiltCentreAngle, speed=200, velocity=20)
    print("------------------------------------------------")
    print("Look Bottom Left")
    setTargetRotationAngle(roll, tilt, rollAngle=roll.fullRotationAngle, tiltAngle=tiltCentreAngle+45, speed=200, velocity=20)
    print("------------------------------------------------")
    print("Look Top Left")
    setTargetRotationAngle(roll, tilt, rollAngle=0, tiltAngle=tiltCentreAngle-45, speed=200, velocity=20)
    # setTargetRotation(tilt, roll, tiltAngle=_tiltAngle, rollAngle=_rollAngle, speed=200, velocity=50)

    

# setupRoutine()

    




# stage2 = Trinamic.TMCM1110("COM8")
# stage1.close()
# stage2.close()