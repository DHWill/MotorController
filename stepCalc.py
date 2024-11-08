STEP_ANGLE = 1.80
U_STEP = 256

def angleToMicrostep(angle):
    ret = angle /STEP_ANGLE
    ret *= U_STEP
    return ret

print(angleToMicrostep(90))

#
          MST 0
          MST 1
          SAP 1, 0, 0
          SAP 1, 1, 0
          SAP 5, 0, 100
          SAP 5, 1, 100
          SAP 0, 0, 9558
          SAP 0, 1, 9558
          SAP 0, 0, 0
          SAP 0, 1, 0
          SAP 7, 0, 8


