from globals import *
from ControllerArm import ControllerArm, MotorController
import threading


#Roll = Serial Adress 0
#Tilt = Serial Adress 1

#Roll Motor Is Master Axes
#Tilt Angle = Roll + Tilt Angle

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

# self, rollAngle:float = 0, tiltAngle:float = 0, speed:int = 100, velocity:int = 50
rotationSequences = [
    (90, -45, 200, 20 ),
    (-85, 45, 200, 20 ),
    (0, 0, 200, 20 ),
    (85, 45, 200, 20 ),
    (-85, -45, 200, 20 ),
]


if(__name__ == "__main__"):
    connected_arms = get_paired_motors()
    setup_pool = []
    for arm in connected_arms:
        t = threading.Thread(target=arm.setupRoutine)
        setup_pool.append(t)
    
    for t in setup_pool:
        t.start()

    settingUp = True
    while settingUp:
        settingUp = False
        for arm in connected_arms:
            if(arm.isSetup == False):
                settingUp = True
    
    for t in setup_pool:
        t.join()
    
    for arm in connected_arms:
        arm.setArmLimitSwitches(False)
        


    

    
    while True:
        for arm in connected_arms:
            if(arm.getPositionReached()):
                _rollAngle, _tiltAngle, _speed, _velocity = rotationSequences[arm.sequencePosition]
                print("_rollAngle: ", _rollAngle, "_tiltAngle: ", _tiltAngle, " _speed: ", _speed, "_velocity: ", _velocity)
                arm.setTargetRotationAngle(rollAngle=_rollAngle, tiltAngle=_tiltAngle,speed=_speed,velocity=_velocity)
                arm.sequencePosition += 1 
                arm.sequencePosition %= len(rotationSequences)
    


    
    

    
    







# #Limit to +/- 45Deg
# tiltCentreAngle = tilt.fullRotationAngle / 2.

# #Centre Point 
# setTargetRotationAngle(roll, tilt, rollAngle=roll.fullRotationAngle/2, tiltAngle=tiltCentreAngle, speed=200, velocity=50)

# while(True):

#     print("------------------------------------------------")
#     print("Look Top Left Neg")
#     setTargetRotationAngle(roll, tilt, rollAngle=roll.fullRotationAngle, tiltAngle=tiltCentreAngle-45, speed=200, velocity=20)
#     print("------------------------------------------------")
#     print("Look Top Left")
#     setTargetRotationAngle(roll, tilt, rollAngle=0, tiltAngle=tiltCentreAngle+45, speed=200, velocity=20)
#     print("------------------------------------------------")
#     print("Look Centre")
#     setTargetRotationAngle(roll, tilt, rollAngle=roll.fullRotationAngle/2, tiltAngle=tiltCentreAngle, speed=200, velocity=20)
#     print("------------------------------------------------")
#     print("Look Bottom Right")
#     setTargetRotationAngle(roll, tilt, rollAngle=roll.fullRotationAngle, tiltAngle=tiltCentreAngle+45, speed=200, velocity=20)
#     print("------------------------------------------------")
#     print("Look Top Left")
#     setTargetRotationAngle(roll, tilt, rollAngle=0, tiltAngle=tiltCentreAngle-45, speed=200, velocity=20)
#     # setTargetRotation(tilt, roll, tiltAngle=_tiltAngle, rollAngle=_rollAngle, speed=200, velocity=50)

    

# setupRoutine()

    




# stage2 = Trinamic.TMCM1110("COM8")
# stage1.close()
# stage2.close()