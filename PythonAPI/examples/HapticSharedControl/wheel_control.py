import sys

sys.path.append("../logidrivepy")
import gc
import time
from pprint import pprint

import numpy as np
import pandas as pd
from logidrivepy import LogitechController

MAX_ANGLE_RANGE = 900


class WheelController:
    def __init__(self):
        self.controller = LogitechController()
        self.controller_index = 0
        self.controller.steering_initialize()
        self.controller.set_operating_range(index=self.controller_index, range=MAX_ANGLE_RANGE)
        print("\n---Logitech Controller Initialized---")
        print("Is connected:", self.controller.is_connected(index=self.controller_index))
        print(
            "Force feedback available:",
            self.controller.has_force_feedback(index=self.controller_index),
        )
        print("Operating range:", self.controller.set_operating_range(index=self.controller_index, range=MAX_ANGLE_RANGE))
        
    def play_spring_force(
        self, offset_percentage, saturation_percentage=0.5, coefficient_percentage=1.0
    ):  
        self.controller.logi_update()
        self.controller.play_spring_force(
            index=self.controller_index,
            offset_percentage=offset_percentage,
            saturation_percentage=saturation_percentage,
            coefficient_percentage=coefficient_percentage,
        )
        self.controller.logi_update()

    def stop_spring_force(self):
        self.controller.stop_spring_force(index=self.controller_index)

    def get_state_engines(self):
        # cspell: ignore rgdw
        state_contents = self.controller.get_state_engines(self.controller_index).contents
        return {
            "lX": state_contents.lX,
            "lY": state_contents.lY,
            "lZ": state_contents.lZ,
            "lRx": state_contents.lRx,
            "lRy": state_contents.lRy,
            "lRz": state_contents.lRz,
            "rglSlider": list(state_contents.rglSlider),
            "rgdwPOV": list(state_contents.rgdwPOV),
            "rgbButtons": list(state_contents.rgbButtons),
            "lVX": state_contents.lVX,
            "lVY": state_contents.lVY,
            "lVZ": state_contents.lVZ,
            "lVRx": state_contents.lVRx,
            "lVRy": state_contents.lVRy,
            "lVRz": state_contents.lVRz,
            "rglVSlider": list(state_contents.rglVSlider),
            "lAX": state_contents.lAX,
            "lAY": state_contents.lAY,
            "lAZ": state_contents.lAZ,
            "lARx": state_contents.lARx,
            "lARy": state_contents.lARy,
            "lARz": state_contents.lARz,
            "rglASlider": list(state_contents.rglASlider),
            "lFX": state_contents.lFX,
            "lFY": state_contents.lFY,
            "lFZ": state_contents.lFZ,
            "lFRx": state_contents.lFRx,
            "lFRy": state_contents.lFRy,
            "lFRz": state_contents.lFRz,
            "rglFSlider": list(state_contents.rglFSlider),
        }

    def get_angle(self, translate=True):
        """
        Retrieves the angle of the wheel control.
        Args:
            translate (bool): If True, the angle is normalized to the range [-1.0, 1.0].
                              If False, the raw angle value is returned.
        Returns:
            float: The angle of the wheel control. If `translate` is True, the value is
                   normalized to the range [-1.0, 1.0]. If `translate` is False, the raw
                   angle value is returned.
        """

        if not translate:
            return self.get_state_engines()["lX"]
        else:
            return np.clip(self.get_state_engines()["lX"], -32767.0, 32767.0) / 32767.0

    def get_acceleration_pedal(self, translate=True):
        """
        Retrieves the current state of the acceleration pedal.
        Args:
            translate (bool): If True, translates the raw pedal value to a normalized range [0, 1].
                              If False, returns the raw pedal value.
        Returns:
            float: The state of the acceleration pedal. If `translate` is True, returns a normalized
                   value between 0 and 1. If `translate` is False, returns the raw value from the engine state.
        """

        if not translate:
            return self.get_state_engines()["lY"]
        else:
            return abs(self.get_state_engines()["lY"] - 32767.0) / 65535.0

    def get_brake_pedal(self, translate=True):
        """
        Retrieves the current state of the brake pedal.
        Args:
            translate (bool): If True, translates the raw pedal value to a normalized range [0, 1].
                              If False, returns the raw pedal value.
        Returns:
            float: The state of the brake pedal. If `translate` is True, returns a normalized
                   value between 0 and 1. If `translate` is False, returns the raw value from the engine state.
        """

        if not translate:
            return self.get_state_engines()["lRZ"]
        else:
            return abs(self.get_state_engines()["lRZ"] - 32767.0) / 65535.0

    def get_DPad(self, translate=True):
        """
        Retrieves the current state of the DPad.
        Args:
            translate (bool): If True, translates the raw DPad value to a normalized range [0, 1].
                              If False, returns the raw DPad value.
        Returns:
            float: The state of the DPad. If `translate` is True, returns a normalized
                   value between 0 and 1. If `translate` is False, returns the raw value from the engine state.
        """

        if not translate:
            return self.get_state_engines()["rgdwPOV"][0]
        else:
            return abs(self.get_state_engines()["rgdwPOV"][0] - 32767.0) / 65535.0

    def exit(self):
        self.controller.steering_shutdown()
        del self.controller
        gc.collect()


def spin_controller_full_test(controller):
    df = {}

    for sat in range(10, 101, 5):
        print(f"Saturation: {sat}", end=" | ")
        for coef in range(10, 101, 5):
            print(f"Coefficient: {coef}", end="\n")
            for offset in range(-100, 101, 1):
                print(f"Offset: {offset}", end="\n")
                # -i * 45 + 90
                # -4 ~ -18
                # -45 ~ -200
                controller.play_spring_force(
                    offset_percentage=offset,
                    saturation_percentage=sat,
                    coefficient_percentage=coef,
                )

                state = controller.get_state_engines()
                # export to csv
                df.setdefault("timestamp", []).append(str(time.time()))
                for key, value in vars(state).items():
                    df.setdefault(key, []).append(value)

                time.sleep(0.3)
            time.sleep(1)
        time.sleep(2)

    df = pd.DataFrame().from_dict(df)
    df.to_excel("spin_test_offset.xlsx", index=False)
    controller.stop_spring_force()


def spin_controller_forward_reverse_test(controller):
    #NOTE: why the offset and measured is not the same?
    time.sleep(3)
    print("Forward test")
    df = {}
    print("--------------------")
    
    for offset in range(-100, 101, 10):
        print(f"//=====\nOffset: {offset}", end="\n")
        # -i * 45 + 90
        # -4 ~ -18
        # -45 ~ -200
        print(f"Angle before: {round(controller.get_angle() * 450, 1)}", end=" --> ")
        controller.play_spring_force(
            offset_percentage=offset,
            saturation_percentage=100,
            coefficient_percentage=100,
        )

        time.sleep(0.5)
        state = controller.get_state_engines()

        # export to csv
        df.setdefault("timestamp", []).append(str(time.time()))
        for key, value in state.items():
            df.setdefault(key, []).append(value)

        print(f"Angle after: {round(controller.get_angle() * 450, 1)}")
        print(f"Acceleration: {controller.get_acceleration_pedal()}")
        time.sleep(1)

    controller.stop_spring_force()
    time.sleep(5)
    
    print("Reverse test")

    for offset in range(100, -101, -10):
        print(f"//=====\nOffset: {offset}", end="\n")
        # -i * 45 + 90
        # -4 ~ -18
        # -45 ~ -200
        print(f"Angle before: {round(controller.get_angle() * 450, 1)}", end=" --> ")
        controller.play_spring_force(
            offset_percentage=offset,
            saturation_percentage=100,
            coefficient_percentage=100,
        )

        time.sleep(0.5)
        state = controller.get_state_engines()
        
        # export to csv
        df.setdefault("timestamp", []).append(str(time.time()))
        for key, value in state.items():
            df.setdefault(key, []).append(value)

        print(f"Angle after: {round(controller.get_angle() * 450, 1)}")
        print(f"Acceleration: {controller.get_acceleration_pedal()}")
        time.sleep(1)

    controller.stop_spring_force()
    print(f"Angle final: {controller.get_angle() * 450}")
    time.sleep(1)
    df = pd.DataFrame().from_dict(df)
    df.to_excel("./data/wheel_logs/spin_test_offset_backward.xlsx", index=False)
    print("Forward reverse test done.")
    time.sleep(1)
    controller.exit()


def spin_test():
    controller = WheelController()
    spin_controller_forward_reverse_test(controller)
    print("Spin test passed.\n")


if __name__ == "__main__":
    spin_test()
    print("Done.")