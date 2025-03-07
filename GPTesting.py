from globals import *
from ControllerArm import ControllerArm, MotorController
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



# motors = get_all_motors()
# _motor = motors[0]
# _motor.setupDefaults()
# while True:
#     print(_motor.getGPI(port=1))
# # _motor.findHome(gpi_pin=0)
    
_controllerArm = get_paired_motors()
controllerArm = _controllerArm[0]

# while True:
#     print(": " + str(controllerArm.tiltMotor.getGPI(port=2)))
#     time.sleep(0.05)

controllerArm.homeMotors()

while controllerArm.isHoming == True:
    pass

