from pylablib.devices import Trinamic
import serial.tools.list_ports
import ctypes
from ctypes import wintypes
import time
import random as rand

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

print(get_usb_com_ports_with_serial_numbers())

#SAP 171, 0, 0 // smartEnergy current up step
#SAP 181, 0, 14 // stop-on-stall velocity
#SAP 170, 0, 0 // smartEnergy hysteresis
#SAP 6, 0, 200 // Maximum current
#SAP 168, 0, 0 // smartEnergy current minimum (SEIMIN)
#SAP 182, 0, 0 // smartEnergy threshold speed
#SAP 173, 0, 1 // stallGuard2 filter enable
#SAP 172, 0, 0 // smartEnergy hysteresis start
#SAP 174, 0, 16 // stallGuard2 threshold
#SAP 7, 0, 16 // Standby current
#SAP 169, 0, 0 // smartEnergy current down step
#SAP 183, 0, 0 // smartEnergy slow run current
#SAP 7, 0, 127    //set Standby current



STEP_ANGLE = 1.80
U_STEP = 256

def angleToMicrostep(angle):
    ret = angle /STEP_ANGLE
    ret *= U_STEP
    ret = int(ret)
    return ret

FULL_ROTATION = angleToMicrostep(360)


# stage1 = Trinamic.TMCM1110("COM4")
# stage2 = Trinamic.TMCMx110("COM4")

MAX_SPEED = 200
MAX_ACCELERATION = 10
class MotorController():
    def __init__(self, comport):
        self.fullRotation = 0
        self.motor = Trinamic.TMCM1110(comport)
        self.stallguardThreshold = 20
        self.name = ""

    def setupDefaults(self):
        self.motor.set_microstep_resolution(256)
        self.motor.set_axis_parameter(parameter=4, value=MAX_SPEED)
        self.motor.set_axis_parameter(parameter=5, value=MAX_ACCELERATION)
        self.motor.set_axis_parameter(parameter=6, value=255) #max current
        self.motor.set_axis_parameter(parameter=7, value=8) #standby current
        self.motor.set_axis_parameter(axis=0, parameter=173, value=1)   #stallGuard Filter
        self.motor.set_axis_parameter(axis=0, parameter=174, value=self.stallguardThreshold)   #stallGuard threshold
        self.motor.set_axis_parameter(axis=0, parameter=181, value=0)   #stallGuard stop velocity
        self.motor.setup_limit_switches(axis=0, left_enable=True, right_enable=True)
        self.motor.stop()

    def centreMotor(self):
        self.motor.move_to(-FULL_ROTATION, axis=0)
        while(self.motor.get_current_speed(axis=0) != 0):
            time.sleep(0.01)
        self.motor.stop()
        self.motor.set_position_reference(axis=0, pos=0)
        self.motor.move_to(FULL_ROTATION, axis=0)
        while(self.motor.get_current_speed(axis=0) != 0):
            print(self.motor.get_current_speed(axis=0))
            time.sleep(0.01)
        print("stop")
        self.motor.stop()

        
        self.fullRotation = self.motor.get_position(axis=0)
        time.sleep(1)
        self.motor.move_to(self.fullRotation/2, axis=0)
        while(self.motor.get_current_speed(axis=0) != 0):
            print(self.motor.get_current_speed(axis=0))
            time.sleep(0.01)
        
        self.motor.stop()

    
    def lockMotor(self):
        self.motor.set_axis_parameter(parameter=7, value=128) #standby current
        self.motor.stop()
    
    def freeMotor(self):
        self.motor.set_axis_parameter(parameter=7, value=0) #standby current
        self.motor.stop()
    
    # def setStallGuard(self, sg: int = 8):
    #     self.motor.set_axis_parameter(axis=0, parameter=174, value=self.stallguardThreshold)   #stallGuard threshold



tilt = MotorController(comport="COM3")
roll = MotorController(comport="COM4")
#Roll Motor Is Master Axes
#Tilt Angle = Roll + Tilt Angle
tilt.name = "tilt"
roll.name = "roll"


def setMotorTaget(_motor: MotorController = None, angle = "", speed:int = MAX_SPEED, acceleration:int = MAX_ACCELERATION):
    _motor.motor.set_axis_parameter(parameter=4, value=speed)
    _motor.motor.set_axis_parameter(parameter=5, value=acceleration)
    _motor.motor.move_to(angleToMicrostep(angle))
    print("Motor", _motor.name, "Angle: ", angleToMicrostep(angle), "Speed: ", speed, "Velocity: ", acceleration)

#Get Max Speed PP/s to make overall roattion time
def setTargetRotation(rollAngle:float = 0, tiltAngle:float = 0, speed:int = 100, velocity:int = 50):
    
    #in here need current angle to compare against target angle
    _rollAngle = rollAngle
    _tiltAngle = tiltAngle
    _tiltAngle += _rollAngle    #roll is Master, and locked on axis
    _rollSpeed = speed
    _tiltSpeed = speed
    _rollVelocity = velocity
    _tiltVelocity = velocity
    mult = 1
    #Match Up speeds
    if(_rollAngle > _tiltAngle):
        mult = (_rollAngle / _tiltAngle)
        _tiltSpeed = speed * mult
        _tiltVelocity = velocity * mult
    elif(_rollAngle < _tiltAngle):
        mult = (_rollAngle / _tiltAngle)
        _rollSpeed = speed * mult
        _rollVelocity = velocity * mult
    
    
    setMotorTaget(tilt, int(_tiltAngle), _tiltSpeed, _tiltVelocity)
    setMotorTaget(roll, int(_rollAngle), _rollSpeed, _rollVelocity)

    atPosition = False
    currentTiltPos = tilt.motor.get_position()
    currentRollPos = roll.motor.get_position()

    while(atPosition == False):

        print("tilt: ", tilt.motor.get_position(), "roll", roll.motor.get_position())
        if((tilt.motor.get_position() == angleToMicrostep(_tiltAngle)) and (roll.motor.get_position() == angleToMicrostep(_rollAngle))):
            atPosition = True
        pass



    
    print("positionReached")
    return


def setupRoutine():
    roll.setupDefaults()
    tilt.setupDefaults()
    tilt.freeMotor()
    roll.centreMotor()
    roll.lockMotor()
    tilt.setupDefaults()
    tilt.centreMotor()
    roll.setupDefaults()
    tilt.setupDefaults()

def zeroPositionReference():
    roll.motor.set_position_reference(axis=0, pos=0)
    tilt.motor.set_position_reference(axis=0, pos=0)

def goToZero():
    roll.motor.move_to(0)
    tilt.motor.move_to(0)
    while((roll.motor.get_position() != int(0) or tilt.motor.get_position() != int(0))):
        print("zeroing motors", tilt.motor.get_position(), roll.motor.get_position())
        time.sleep(0.005)
    return True

def fixTiltRockPan(speed: int = MAX_SPEED, positionRockTo: int = 0, positionRockBack: int = 0):
    tilt.lockMotor()
    
    spaceFromEdge = U_STEP * 20
    positionRockTo = pan.fullRotation - spaceFromEdge
    positionRockBack = spaceFromEdge
    pan.motor.set_axis_parameter(parameter=4, value=speed)
    
    pan.motor.move_to(positionRockTo)
    while(pan.motor.get_position() != positionRockTo):
        print("forth", pan.motor.get_position())
        time.sleep(0.005)
        # if(pan.motor.get_current_speed() < speed - 50):
            # break
    pan.motor.stop()
    time.sleep(0.05)
    pan.motor.move_to(positionRockBack)
    while(pan.motor.get_position() != positionRockBack):
        print("Back", pan.motor.get_position())
        time.sleep(0.005)
        # if(pan.motor.get_current_speed() < speed - 50):
            # break


zeroPositionReference()
tiltAngle = 0
while(True):
    tiltAngle = rand.randrange(-45, 45, 1)
    rollAngle = rand.randrange(-45, 45, 1)
    setTargetRotation(tiltAngle, rollAngle, 200, 50)


# setupRoutine()

    




# stage2 = Trinamic.TMCM1110("COM8")
# stage1.close()
# stage2.close()