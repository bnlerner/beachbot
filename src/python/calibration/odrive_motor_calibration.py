from __future__ import print_function

import odrive # type: ignore[import-untyped]
from odrive import enums
import time
import math



def main(drive: odrive.ODrive) -> None:
    # TODO: Implement the pythononic way to calibrate the ODrive
    pass

    # Calibrate motor and wait for it to finish
    # print("starting calibration...")
    # my_drive.axis0.requested_state = enums.AxisState.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    # while my_drive.axis0.current_state != AXIS_STATE_IDLE:
    #     time.sleep(0.1)

    # my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    # # To read a value, simply read the property
    # print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

    # # Or to change a value, just assign to the property
    # my_drive.axis0.controller.input_pos = 3.14
    # print("Position setpoint is " + str(my_drive.axis0.controller.pos_setpoint))

    # # And this is how function calls are done:
    # for i in [1,2,3,4]:
    #     print('voltage on GPIO{} is {} Volt'.format(i, my_drive.get_adc_voltage(i)))

    # # A sine wave to test
    # t0 = time.monotonic()
    # while True:
    #     setpoint = 4.0 * math.sin((time.monotonic() - t0)*2)
    #     print("goto " + str(int(setpoint)))
    #     my_drive.axis0.controller.input_pos = setpoint
    #     time.sleep(0.01)

if __name__ == "__main__":
    # Find a connected ODrive (this will block until you connect one)
    print("finding an odrive...")
    random_odrive = odrive.find_any()
    main(random_odrive)