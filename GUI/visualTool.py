import dearpygui.dearpygui as dpg
import random
import time
import threading
from pytrinamic.modules import TMCM1110
from ControllerSet import ControllerSet

class MotorDataRequest:
    def __init__(self, _plotLabel: str, _axis_parameter: str, _axis_parameter_max: str, _is_module_request: bool = False, _reply_is_signed:bool=True):
        self.plotLabel = _plotLabel
        self.axis_parameter = _axis_parameter
        self.axis_parameter_max = _axis_parameter_max
        self.is_module_request = _is_module_request
        self.axis_parameter_max_value = 1
        self.scalar = 1.
        self.reply_is_signed = _reply_is_signed
        # self.yData = [0] * BUFFER_SIZE 
        self.yData = []
        self.xData = []

class Visual():
    def __init__(self, fps:int = 20) -> None:
        self.roll_data_requests = [
            # ControllerDataType("position", TMCM1110._MotorTypeA.AP.ActualPosition, False),
            MotorDataRequest(_plotLabel="roll.current", _axis_parameter=TMCM1110._MotorTypeA.AP.SmartEnergyActualCurrent, _axis_parameter_max=TMCM1110._MotorTypeA.AP.MaxCurrent, _is_module_request=True, _reply_is_signed=False),
            MotorDataRequest(_plotLabel="roll.velocity", _axis_parameter=TMCM1110._MotorTypeA.AP.ActualVelocity, _axis_parameter_max=TMCM1110._MotorTypeA.AP.MaxVelocity, _is_module_request=False, _reply_is_signed=False),
            MotorDataRequest(_plotLabel="roll.acceleration", _axis_parameter=TMCM1110._MotorTypeA.AP.ActualAcceleration, _axis_parameter_max=TMCM1110._MotorTypeA.AP.MaxAcceleration, _is_module_request=False, _reply_is_signed=False)
        ]
        
        self.tilt_data_requests = [
            MotorDataRequest("tilt.current", _axis_parameter=TMCM1110._MotorTypeA.AP.SmartEnergyActualCurrent, _axis_parameter_max=TMCM1110._MotorTypeA.AP.MaxCurrent, _is_module_request=True, _reply_is_signed=False),
            MotorDataRequest("tilt.velocity", _axis_parameter=TMCM1110._MotorTypeA.AP.ActualVelocity, _axis_parameter_max=TMCM1110._MotorTypeA.AP.MaxVelocity, _is_module_request=False, _reply_is_signed=False),
            MotorDataRequest("tilt.acceleration", _axis_parameter=TMCM1110._MotorTypeA.AP.ActualAcceleration, _axis_parameter_max=TMCM1110._MotorTypeA.AP.MaxAcceleration, _is_module_request=False, _reply_is_signed=False)
        ]
        # self.lock = threading.Lock()
        # self.controller_set = [self.rollData, self.tiltData]
        self.isNewData = False
        self.start_time = time.time()
        self.ms_interval = float(1/fps)  # Update interval (20 FPS)
        self.controller_set = None

    
    # def set_motor(self, motor:TMCM1110._MotorTypeA) -> None:
    def start_plot(self)-> None:
        # for motor in self.controller_set:
        for data in self.roll_data_requests:
            data.xData.clear()
            data.yData.clear()
        
        for data in self.tilt_data_requests:
            data.xData.clear()
            data.yData.clear()

        self.start_time = time.time()
    
    def controller_data_request(self, controller:TMCM1110._MotorTypeA, data_request: MotorDataRequest, update_max_params = False) -> int:
        value = 0
        if(update_max_params):
            data_request.axis_parameter_max_value = controller.get_axis_parameter(ap_type=data_request.axis_parameter_max, signed=data_request.reply_is_signed)
        else:
            value = controller.get_axis_parameter(ap_type=data_request.axis_parameter, signed=data_request.reply_is_signed) / data_request.axis_parameter_max_value
        return value

    def module_data_request(self, module:TMCM1110, data_request: MotorDataRequest, update_max_params = False) -> int:
        value = 0
        if(update_max_params):
            data_request.axis_parameter_max_value = module.get_axis_parameter(axis=0, ap_type=data_request.axis_parameter_max, signed=data_request.reply_is_signed)
        else:
            value = module.get_axis_parameter(axis=0, ap_type=data_request.axis_parameter, signed=data_request.reply_is_signed) / data_request.axis_parameter_max_value
        return value
    

    
    def get_data(self, controller_set:ControllerSet) -> None:
        self.isNewData = True
        if(controller_set != self.controller_set):    #If different motors update max parameters
            self.controller_set = controller_set
            for data in self.roll_data_requests:
                if(data.is_module_request):
                    self.module_data_request(module=self.controller_set.rollMotorModule, data_request= data, update_max_params=True)
                else:
                    self.controller_data_request(controller=self.controller_set.rollMotor, data_request= data, update_max_params=True)

            
            for data in self.tilt_data_requests:
                if(data.is_module_request):
                    self.module_data_request(module=self.controller_set.tiltMotorModule, data_request= data, update_max_params=True)
                else:
                    self.controller_data_request(controller=self.controller_set.tiltMotor, data_request= data, update_max_params=True)
        

        for data in self.roll_data_requests:
            if(data.is_module_request):
                value = self.module_data_request(module=self.controller_set.rollMotorModule, data_request= data, update_max_params=False)
            else:
                value = self.controller_data_request(controller=self.controller_set.rollMotor, data_request= data, update_max_params=False)
            data.yData.append(value)
            data.xData.append(time.time() - self.start_time)
        
        for data in self.tilt_data_requests:
            if(data.is_module_request):
                value = self.module_data_request(module=self.controller_set.tiltMotorModule, data_request= data, update_max_params=False)
            else:
                value = self.controller_data_request(controller=self.controller_set.tiltMotor, data_request= data, update_max_params=False)
            data.yData.append(value)
            data.xData.append(time.time() - self.start_time)

            # if len(data.yData) > BUFFER_SIZE:
            #     data.yData.pop(0)
            #     data.xData.pop(0)
            # data.yData.pop(0)

    def update_plot(self) -> None:
        for data in self.roll_data_requests:
            dpg.set_value(data.plotLabel, [data.xData, data.yData])  # Update each line
        
        for data in self.tilt_data_requests:
            dpg.set_value(data.plotLabel, [data.xData, data.yData])  # Update each line

        if self.roll_data_requests[0].xData:  
            latest_x = self.roll_data_requests[0].xData[-1]
            window_size = 20  # Adjust this value to control how much history is visible
            dpg.set_axis_limits("roll_x_axis", latest_x - window_size, latest_x)
            dpg.set_axis_limits("tilt_x_axis", latest_x - window_size, latest_x)
        
    
    def init_gui(self) -> None:
        dpg.create_context()
        w_wid = 700
        w_hig = 700
        with dpg.window(label="Controller Set", width=w_wid, height=w_hig):
            with dpg.plot(label="roll_motor", width=w_wid, height=w_hig/2):
                dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="roll_x_axis",pan_stretch=True)
                with dpg.plot_axis(dpg.mvYAxis, label="Value", tag="roll_y_axis",pan_stretch=True):
                    for data in self.roll_data_requests:
                        dpg.add_line_series(data.xData, data.yData, label=data.plotLabel, tag=data.plotLabel)
                dpg.add_plot_legend()
            
            with dpg.plot(label="tilt_motor", width=w_wid, height=w_hig/2):
                dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag="tilt_x_axis",pan_stretch=True)
                with dpg.plot_axis(dpg.mvYAxis, label="Value", tag="tilt_y_axis",pan_stretch=True):
                    for data in self.tilt_data_requests:
                        dpg.add_line_series(data.xData, data.yData, label=data.plotLabel, tag=data.plotLabel)
                dpg.add_plot_legend()
                       
        dpg.set_axis_limits("roll_y_axis", -1.25, 1.25)  # Set Y-axis range from -1 to 1
        dpg.set_axis_limits_auto("roll_x_axis")
        dpg.set_axis_limits("tilt_y_axis", -1.25, 1.25)  # Set Y-axis range from -1 to 1
        dpg.set_axis_limits_auto("tilt_x_axis")

        dpg.create_viewport(title="Motor_Controller", width=w_wid+100, height=w_hig+50) #Get dpg bar wid
        dpg.setup_dearpygui()
        dpg.show_viewport()
        # Keep updating the GUI until it closes
        while dpg.is_dearpygui_running():
            if(self.isNewData):
                self.update_plot()  # Call update function
                self.isNewData = False
            
            dpg.render_dearpygui_frame()  # **Render new frame**
            time.sleep(self.ms_interval)  # **Control update rate**
    

    def kill_gui(self) -> None:
        dpg.destroy_context()