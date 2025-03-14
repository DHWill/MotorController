import dearpygui.dearpygui as dpg
import random
import time
import threading
from pytrinamic.modules import TMCM1110
# Constants
DURATION = 1  # Seconds to display
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
        self.scalar = 1.
        # self.yData = [0] * BUFFER_SIZE 
        self.yData = []
        self.xData = []

class Visual():
    def __init__(self) -> None:
        self.data = [
            # ControllerDataType("position", TMCM1110._MotorTypeA.AP.ActualPosition, False),
            ControllerDataType("current", TMCM1110._MotorTypeA.AP.SmartEnergyActualCurrent, TMCM1110._MotorTypeA.AP.MaxCurrent, True),
            ControllerDataType("velocity", TMCM1110._MotorTypeA.AP.ActualVelocity, TMCM1110._MotorTypeA.AP.MaxVelocity, False),
            ControllerDataType("acceleration", TMCM1110._MotorTypeA.AP.ActualAcceleration, TMCM1110._MotorTypeA.AP.MaxAcceleration, False),
        ]
        # self.lock = threading.Lock()
        self.motor = None
        self.isNewData = False
        self.start_time = time.time()
    
    # def set_motor(self, motor:TMCM1110._MotorTypeA) -> None:
    
    def get_data(self, motor:TMCM1110._MotorTypeA) -> None:
        self.isNewData = True
        if(motor != self.motor):    #If different motors update max parameters
            self.motor = motor
            for data in self.data:
                data.axis_parameter_max_value = self.motor.get_axis_parameter(ap_type=data.axis_parameter_max, signed=True)

        for data in self.data:
            value = self.motor.get_axis_parameter(ap_type=data.axis_parameter, signed=True) / data.axis_parameter_max_value
            print(data.plotLabel, value)
            data.yData.append(value)
            data.xData.append(time.time() - self.start_time)


            # if len(data.yData) > BUFFER_SIZE:
            #     data.yData.pop(0)
            #     data.xData.pop(0)
            # data.yData.pop(0)

    def update_plot(self) -> None:
        for data in self.data:
            dpg.set_value(data.plotLabel, [data.xData, data.yData])  # Update each line
    
        if self.data[0].xData:  # Ensure there's data before updating
            latest_x = self.data[0].xData[-1]  # Get the latest time value
            window_size = 2  # Adjust this value to control how much history is visible
            dpg.set_axis_limits("x_axis", latest_x - window_size, latest_x)
        
    def start_plot(self)-> None:
        for data in self.data:
            data.xData.clear()
            data.yData.clear()
        
        self.start_time = time.time()

    
    def init_gui(self) -> None:
        dpg.create_context(width=700, height=700)
        with dpg.window(label="Motor Controller", width=700, height=700):
            with dpg.plot(label="Motor-Stats", width=700, height=700):
                dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="x_axis",pan_stretch=True)
                with dpg.plot_axis(dpg.mvYAxis, label="Value", tag="y_axis",pan_stretch=True):
                    for data in self.data:
                        dpg.add_line_series(data.xData, data.yData, label=data.plotLabel, tag=data.plotLabel)
                dpg.add_plot_legend()
        
        dpg.set_axis_limits("y_axis", -1.25, 1.25)  # Set Y-axis range from -1 to 1
        dpg.set_axis_limits_auto("x_axis")

        dpg.create_viewport(title="Motor_Controller", width=700, height=700)
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