from globals import *
from ControllerSet import ControllerSet
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1110
from GUI.visualTool import Visual
import threading
import time


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
            controller_arm__array.append(ControllerSet(_rollMotorModule=_rollController, _tiltMotorModule= _tiltController, _armID = armId))
            armId += 1
    
    return controller_arm__array


def wait_all_inposition(_connected_arms:list[ControllerSet]):
    for arm in _connected_arms:
        arm.waitPositionReached()


if(__name__ == "__main__"):
    interface = ConnectionManager("--interface serial_tmcl --port COM36 --data-rate 9600")
    interface_connection = interface.connect()
    connected_arms = get_paired_motors_on_bus(bus_connection=interface_connection)
    controllerSet1 = connected_arms[0]
    
    gui = Visual()
    guiThread = threading.Thread(target=gui.init_gui)
    guiThread.start()


    # gui.set_motor(controllerSet1.rollMotor)
    controllerSet1.rollDisc(_angle =-(360 * 2), _velocity =1000, _accelleration=2000)

    gui.start_plot()
    
    while(controllerSet1.getIsMoving()):
        gui.get_data(controllerSet1.rollMotor)
    
    
    while True:
        time.sleep(1)
        # print("Velocity", controllerSet1.rollMotor.get_actual_velocity())

    #while True:
    #    for sequence in range(len(rotationSequences1)):
    #        wait_all_inposition(connected_arms)
    #        time.sleep(1)
    #        for arm in connected_arms:
    #            if(arm.getPositionReached()):
    #                ra, ta, vel, accel = rotationSequences1[arm.sequencePosition]
    #                print("_rollAngle: ", ra, "_tiltAngle: ", ta, " _speed: ", vel, "_acceleration: ", accel)
    #                arm.setTargetRotationAngle(_rollAngle=ra, _tiltAngle=ta, _velocity=vel, _acceleration=accel)
    #                arm.sequencePosition += 1 
    #                arm.sequencePosition %= len(rotationSequences1)