from globals import *
from ControllerArm import ControllerArm, MotorController
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1110
import time



def get_all_motors() -> list[MotorController]:
    comportList = get_usb_com_ports_with_serial_numbers()
    controllers = []
    for _comport, serial_number in comportList:
        motor = None
        try:
            motor = MotorController(comport=_comport)
        except:
            print("No Motor detected on:", _comport)

        if(motor != None):
            motorID = motor.getControllerSecondaryAddress()
            armID = motor.getControllerPrimaryAddress()
            if(motorID == 0):
                motor.name = "roll"
            elif(motorID == 1):
                motor.name = "tilt"
            controllers.append(motor)
    return controllers


def get_paired_motors() -> list[ControllerArm]:
    comportList = get_usb_com_ports_with_serial_numbers()
    controllers = []
    for _comport, serial_number in comportList:
        motor = None
        try:
            motor = MotorController(comport=_comport)
        except:
            print("No Motor detected on:", _comport)

        if(motor != None):
            motorID = motor.getControllerSecondaryAddress()
            armID = motor.getControllerPrimaryAddress()
            if(motorID == 0):
                motor.name = "roll"
            elif(motorID == 1):
                motor.name = "tilt"
            controllers.append((armID, motorID, motor))
    
    motor_set = []
    for armID, motorID, motor in controllers:
        for armID_1, motorID_1, motor_1 in controllers:
            if((armID_1 == armID) and (motorID != motorID_1)):
                if(motorID == 0):
                    motor_set.append(ControllerArm(_rollMotor = motor, _tiltMotor= motor_1, _armID = armID))
    
    return motor_set

#Master ID is 2 start from 3:
#3 -> 256
NODE_ID_START = 3
NODE_ID_END = 5


def get_paired_motors_on_bus(bus_connection:ConnectionManager = None) -> list[ControllerArm]:
    print(bus_connection)
    controller_arm__array = []
    armId = 0
    for id in range(NODE_ID_START, NODE_ID_END, 2):
        try:
            rollMotor = MotorController(bus_connection, id)
            print("rollMotor: " , rollMotor.getControllerPrimaryAddress())
        except:
            print("couldn't find roll motor:", id)
        
        try:
            tiltMotor = MotorController(bus_connection, id+1)
            print("tiltMotor: " , tiltMotor.getControllerPrimaryAddress())
        except:
            print("couldn't find tilt motor:", id+1)
        if(rollMotor and tiltMotor):
            controller_arm__array.append(ControllerArm(_rollMotor = rollMotor, _tiltMotor= tiltMotor, _armID = armId))
            armId += 1
    
    return controller_arm__array



# motors = get_all_motors()
# _motor = motors[0]
# _motor.setupDefaults()
# while True:
#     print(_motor.getGPI(port=1))
# # _motor.findHome(gpi_pin=0)
    
interface = ConnectionManager("--interface serial_tmcl --port COM36 --data-rate 9600")
interface_connection = interface.connect()
_controllerArm = get_paired_motors_on_bus(bus_connection=interface_connection)
controllerArm = _controllerArm[0]
controllerArm.homeMotors()

# while True:
#     print(": " + str(controllerArm.tiltMotor.getGPI(port=2)))
#     time.sleep(0.05)


# while controllerArm.isHoming == True:
#     pass

