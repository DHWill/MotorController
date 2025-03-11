from globals import *
from ControllerSet import ControllerSet
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1110
import threading
import time



#Roll = Serial Adress 0
#Tilt = Serial Adress 1

#Roll Motor Is Master Axes
#Tilt Angle = Roll + Tilt Angle

def get_paired_motors() -> list[ControllerSet]:
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
                    motor_set.append(ControllerSet(_rollMotor = motor, _tiltMotor= motor_1, _armID = armID))
    
    return motor_set


#Master ID is 2 start from 3:
#3 -> 256
NODE_ID_START = 3
NODE_ID_END = 5

def get_paired_motors_on_bus(bus_connection:ConnectionManager = None) -> list[ControllerSet]:
    print(bus_connection)
    controller_arm__array = []
    armId = 0
    for id in range(NODE_ID_START, NODE_ID_END, 2):
        try:
            _rollController = TMCM1110(bus_connection, module_id=id)
            print("rollMotor: " , _rollController.get_global_parameter(gp_type=TMCM1110.GP0.SerialAddress, bank=0, signed=False))
        except:
            print("couldn't find roll motor:", id)
        
        try:
            _tiltController = TMCM1110(bus_connection, module_id=id+1)
            print("tiltMotor: " , _tiltController.get_global_parameter(gp_type=TMCM1110.GP0.SerialAddress, bank=0, signed=False))
        except:
            print("couldn't find tilt motor:", id+1)

        if(_rollController and _tiltController):
            controller_arm__array.append(ControllerSet(_rollMotorController=_rollController, _tiltMotorController= _tiltController, _armID = armId))
            armId += 1
    
    return controller_arm__array



# motors = get_all_motors()
# _motor = motors[0]
# _motor.setupDefaults()
# while True:
#     print(_motor.getGPI(port=1))
# # _motor.findHome(gpi_pin=0)
rotationSequences1 = [
    (-85, -45, 200, 20 ),
    (90, 45, 200, 20 ),
    (0, 0, 200, 20 ),
    (-85, 45, 200, 20 ),
    (85, -45, 200, 20 ),
]
    
# controllerArm = _controllerArm[0]
# controllerArm.homeMotors()




def wait_all_inposition(_connected_arms:list[ControllerSet]):
    for arm in _connected_arms:
        arm.waitPositionReached()


if(__name__ == "__main__"):
    interface = ConnectionManager("--interface serial_tmcl --port COM36 --data-rate 9600")
    interface_connection = interface.connect()
    connected_arms = get_paired_motors_on_bus(bus_connection=interface_connection)
    setup_pool = []

    # controllerSet1 = connected_arms[0]
    # # print(controllerSet1.rollMotorController.list_features())
    # # controllerSet1.homeMotors()
    
    for arm in connected_arms:
        t = threading.Thread(target=arm.homeMotors)
        setup_pool.append(t)
    
    for t in setup_pool:
        t.start()

    
    # Catch the Rest Setting up 
    settingUp = True
    while settingUp:
        settingUp = False
        for arm in connected_arms:
            if(arm.isHoming == True):
                settingUp = True
    

    # while len(setup_pool) > 0:
    for t in setup_pool:
        t.join()
    
    # for arm in connected_arms:
    #     # arm.setArmLimitSwitches(False)
    #     arm.setTargetRotationAngle(_rollAngle=-90, _tiltAngle=0,_velocity=200,_acceleration=50)
    
    
    
    
    # wait_all_inposition(connected_arms)
    # print("setup")
    # time.sleep(2)

    while True:
        for sequence in range(len(rotationSequences1)):
            wait_all_inposition(connected_arms)
            time.sleep(1)
            for arm in connected_arms:
                if(arm.getPositionReached()):
                    ra, ta, vel, accel = rotationSequences1[arm.sequencePosition]
                    print("_rollAngle: ", ra, "_tiltAngle: ", ta, " _speed: ", vel, "_acceleration: ", accel)
                    arm.setTargetRotationAngle(_rollAngle=ra, _tiltAngle=ta, _velocity=vel, _acceleration=accel)
                    arm.sequencePosition += 1 
                    arm.sequencePosition %= len(rotationSequences1)



        


    # while True:
    #     wait_all_inposition(connected_arms)    
    #     _rollAngle, _tiltAngle, _speed, _velocity = rotationSequences1[arm.sequencePosition]
    #     print("_rollAngle: ", _rollAngle, "_tiltAngle: ", _tiltAngle, " _speed: ", _speed, "_velocity: ", _velocity)
    #     connected_arms[0].setTargetRotationAngle(rollAngle=_rollAngle, tiltAngle=_tiltAngle,speed=_speed,acceleration=_velocity)
    #     # _rollAngle, _tiltAngle, _speed, _velocity = rotationSequences1[arm.sequencePosition]
    #     # connected_arms[1].setTargetRotationAngle(rollAngle=_rollAngle, tiltAngle=_tiltAngle,speed=_speed,acceleration=_velocity)
    #     # print("_rollAngle: ", _rollAngle, "_tiltAngle: ", _tiltAngle, " _speed: ", _speed, "_velocity: ", _velocity)
    #     arm.sequencePosition += 1 
    #     arm.sequencePosition %= len(rotationSequences1)
    
    

    
    







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