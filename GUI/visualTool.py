import dearpygui.dearpygui as dpg
import random
import time
import threading
from pytrinamic.modules import TMCM1110
# Constants
DURATION = 0.5  # Seconds to display
UPDATE_RATE = 0.05  # Update interval (20 FPS)
BUFFER_SIZE = int(DURATION / UPDATE_RATE)  # Number of points in buffer
x_data = [i * UPDATE_RATE for i in range(BUFFER_SIZE)]  # Time values

class ControllerDataType:
    def __init__(self, plotLabel: str, axis_parameter: str, axis_parameter_max: str, isBoard: bool = False):
        self.plotLabel = plotLabel
        self.axis_parameter = axis_parameter
        self.axis_parameter_max = axis_parameter_max
        self.isBoard = isBoard
        self.axis_parameter_max_value = 0
        self.scalar = 0.2
        self.yData = [0] * BUFFER_SIZE 

class Visual():
    def __init__(self) -> None:
        self.data = [
            # ControllerDataType("position", TMCM1110._MotorTypeA.AP.ActualPosition, False),
            ControllerDataType("current", TMCM1110._MotorTypeA.AP.SmartEnergyActualCurrent, TMCM1110._MotorTypeA.AP.MaxCurrent, True),
            ControllerDataType("velocity", TMCM1110._MotorTypeA.AP.ActualVelocity, TMCM1110._MotorTypeA.AP.MaxVelocity, False),
            ControllerDataType("acceleration", TMCM1110._MotorTypeA.AP.ActualAcceleration, TMCM1110._MotorTypeA.AP.MaxAcceleration, False),
        ]
        self.lock = threading.Lock()
        self.motor = None
        self.isNewData = False
    
    def set_motor(self, motor:TMCM1110._MotorTypeA) -> None:
        self.motor = motor
        for data in self.data:
            data.axis_parameter_max_value = abs(self.motor.get_axis_parameter(ap_type=data.axis_parameter_max, signed=True))
    
    def get_data(self) -> None:
        self.isNewData = True
        for data in self.data:
            value = abs(self.motor.get_axis_parameter(ap_type=data.axis_parameter, signed=True)) / data.axis_parameter_max_value
            print(data.plotLabel, value)
            data.yData.append(value)
            data.yData.pop(0) 

    def update_plot(self) -> None:
        for data in self.data:
            dpg.set_value(data.plotLabel, [x_data, data.yData])  # Update each line

    
    def init_gui(self) -> None:
        dpg.create_context()
        with dpg.window(label="Motor Controller", width=700, height=500):
            with dpg.plot(label="Real-Time Data", height=700, width=600):
                dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="x_axis",pan_stretch=True)
                with dpg.plot_axis(dpg.mvYAxis, label="Signal", tag="y_axis"):
                    for data in self.data:
                        dpg.add_line_series(x_data, data.yData, label=data.plotLabel, tag=data.plotLabel)

        dpg.create_viewport(title="Motor_Controller", width=700, height=500)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        # Keep updating the GUI until it closes
        while dpg.is_dearpygui_running():
            if(self.isNewData):
                self.update_plot()  # Call update function
                self.isNewData = False
            
            dpg.render_dearpygui_frame()  # **Render new frame**
            time.sleep(UPDATE_RATE)  # **Control update rate**
    

    def kill_gui(self) -> None:
        dpg.destroy_context()