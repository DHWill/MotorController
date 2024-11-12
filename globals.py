import serial.tools.list_ports

STEP_ANGLE = 1.8
U_STEP = 256
FULL_STEP = (360. / STEP_ANGLE) * U_STEP
MAX_SPEED = 200
MAX_ACCELERATION = 50


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
    

def angleToMicrostep(angle):
    ret = FULL_STEP/360.
    ret *= angle
    ret = int(ret)
    return ret

def microstepToAngle(microstep):
    ret = 360./FULL_STEP
    ret *= microstep
    return ret

