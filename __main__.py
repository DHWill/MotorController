from globals import *
from ControllerSet import ControllerSet
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1110
from GUI.visualTool import Visual
import threading
import time
from RotationSequences.DemoSequences import rotationSequences2, facingSequence, demoSequence
import random


#Master ID is 2 start from 3:
#3 -> 256
NODE_ID_START = 3
NODE_ID_END = 8

CONNECTED_CONTROLLERS = [
    # 3,
    # 4,
    # 5,
    # 8
]

def get_paired_motors_on_bus(bus_connection:ConnectionManager = None) -> list[ControllerSet]:
    print(bus_connection)
    controller_arm__array = []
    armId = 0
    for id in range(NODE_ID_START, NODE_ID_END, 2):
    # for id in CONNECTED_CONTROLLERS:
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
    still_moving = True
    while still_moving:
        for arm in _connected_arms:
            still_moving = False
            if(arm.getPositionReached() == False):
                still_moving = True
                break

def sync_sequences(_connected_arms:list[ControllerSet], _sequence):
    sequence = _sequence
    sequence = (0, 0, 500, 500)
    
    biggest_distance = 0
    target_velocity = 0
    for arm in range(len(_connected_arms)):
        stagedDist = 0
        cra, cta = _connected_arms[arm].get_set_angles()
        ra, ta, vel, accel = sequence[arm]
        rDist = abs(ra - cra)
        tDist = abs(ta - cta)
        if(rDist > tDist):
            stagedDist = rDist
        else:
            stagedDist = tDist
        if(stagedDist > biggest_distance):
            biggest_distance = stagedDist
    

    velocities = []
    mult = float(1./biggest_distance)
    for arm in range(len(_connected_arms)):
        ra, ta, vel, accel = sequence[arm]
        vel *= mult
        accel *= mult
    
    return sequence



        # print("CURRENT: ", "_rollAngle: ", cra, "_tiltAngle: ", cta)
    





if(__name__ == "__main__"):
    interface = ConnectionManager("--interface serial_tmcl --port COM8 --data-rate 9600")
    interface_connection = interface.connect()
    connected_arms = get_paired_motors_on_bus(bus_connection=interface_connection)
    # controllerSet1 = connected_arms[0]

    for disc in connected_arms:
        disc.homeMotors3()
    

    
    # gui = Visual()
    # guiThread = threading.Thread(target=gui.init_gui)
    # guiThread.start()


    # gui.set_motor(controllerSet1.rollMotor)``
#     gui.start_plot()
    
    
    # while True:
    #     time.sleep(1)
    #     # print("Velocity", controllerSet1.rollMotor.get_actual_velocity())


    # while True:
    #     time.sleep(3)

    time.sleep(2)
    while True:
        #random sequences
        for sequence in demoSequence:
            #facing sequence
            wait_all_inposition(connected_arms)
            for disc in connected_arms:
                ra, ta, vel, accel = sequence[disc.armID]
                print("_rollAngle: ", ra, "_tiltAngle: ", ta, " _velocity: ", vel, "_acceleration: ", accel)
                disc.setTargetRotationAngle(_rollAngle=ra, _tiltAngle=ta, _velocity=vel, _acceleration=accel)
            