"""
Joystick controller using OculusReader input.
exaFLOPs
"""

import numpy as np
from collections.abc import Callable
from scipy.spatial.transform import Rotation

import carb
import omni
import time
import math
import ipdb
from ..device_base import DeviceBase

# from .oculus_reader import OculusReader
from .FPS_counter import FPSCounter
from .buttons_parser import parse_buttons
import threading
import os
from ppadb.client import Client as AdbClient
import sys


def eprint(*args, **kwargs):
    RED = "\033[1;31m"
    sys.stderr.write(RED)
    print(*args, file=sys.stderr, **kwargs)
    RESET = "\033[0;0m"
    sys.stderr.write(RESET)


class OculusReader:
    def __init__(
        self,
        ip_address=None,
        port=5555,
        APK_name="com.rail.oculus.teleop",
        print_FPS=False,
        run=True,
    ):
        self.running = False
        self.last_transforms = {}
        self.last_buttons = {}
        self._lock = threading.Lock()
        self.tag = "wE9ryARX"

        self.ip_address = ip_address
        self.port = port
        self.APK_name = APK_name
        self.print_FPS = print_FPS
        if self.print_FPS:
            self.fps_counter = FPSCounter()

        self.device = self.get_device()
        self.install(verbose=False)
        if run:
            self.run()

    def __del__(self):
        self.stop()

    def run(self):
        self.running = True
        self.device.shell(
            'am start -n "com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity" -a android.intent.action.MAIN -c android.intent.category.LAUNCHER'
        )
        self.thread = threading.Thread(
            target=self.device.shell, args=("logcat -T 0", self.read_logcat_by_line)
        )
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, "thread"):
            self.thread.join()

    def get_network_device(self, client, retry=0):
        try:
            client.remote_connect(self.ip_address, self.port)
        except RuntimeError:
            os.system("adb devices")
            client.remote_connect(self.ip_address, self.port)
        device = client.device(self.ip_address + ":" + str(self.port))

        if device is None:
            if retry == 1:
                os.system("adb tcpip " + str(self.port))
            if retry == 2:
                eprint(
                    "Make sure that device is running and is available at the IP address specified as the OculusReader argument `ip_address`."
                )
                eprint("Currently provided IP address:", self.ip_address)
                eprint("Run `adb shell ip route` to verify the IP address.")
                exit(1)
            else:
                self.get_device(client=client, retry=retry + 1)
        return device

    def get_usb_device(self, client):
        try:
            devices = client.devices()
        except RuntimeError:
            os.system("adb devices")
            devices = client.devices()
        for device in devices:
            if device.serial.count(".") < 3:
                return device
        eprint(
            "Device not found. Make sure that device is running and is connected over USB"
        )
        eprint("Run `adb devices` to verify that the device is visible.")
        exit(1)

    def get_device(self):
        # Default is "127.0.0.1" and 5037
        client = AdbClient(host="127.0.0.1", port=5037)
        if self.ip_address is not None:
            return self.get_network_device(client)
        else:
            return self.get_usb_device(client)

    def install(self, APK_path=None, verbose=True, reinstall=False):
        try:
            installed = self.device.is_installed(self.APK_name)
            if not installed or reinstall:
                if APK_path is None:
                    APK_path = os.path.join(
                        os.path.dirname(os.path.realpath(__file__)),
                        "APK",
                        "teleop-debug.apk",
                    )
                success = self.device.install(APK_path, test=True, reinstall=reinstall)
                installed = self.device.is_installed(self.APK_name)
                if installed and success:
                    print("APK installed successfully.")
                else:
                    eprint("APK install failed.")
            elif verbose:
                print("APK is already installed.")
        except RuntimeError:
            eprint("Device is visible but could not be accessed.")
            eprint(
                "Run `adb devices` to verify that the device is visible and accessible."
            )
            eprint(
                'If you see "no permissions" next to the device serial, please put on the Oculus Quest and allow the access.'
            )
            exit(1)

    def uninstall(self, verbose=True):
        try:
            installed = self.device.is_installed(self.APK_name)
            if installed:
                success = self.device.uninstall(self.APK_name)
                installed = self.device.is_installed(self.APK_name)
                if not installed and success:
                    print("APK uninstall finished.")
                    print(
                        'Please verify if the app disappeared from the list as described in "UNINSTALL.md".'
                    )
                    print(
                        "For the resolution of this issue, please follow https://github.com/Swind/pure-python-adb/issues/71."
                    )
                else:
                    eprint("APK uninstall failed")
            elif verbose:
                print("APK is not installed.")
        except RuntimeError:
            eprint("Device is visible but could not be accessed.")
            eprint(
                "Run `adb devices` to verify that the device is visible and accessible."
            )
            eprint(
                'If you see "no permissions" next to the device serial, please put on the Oculus Quest and allow the access.'
            )
            exit(1)

    @staticmethod
    def process_data(string):
        try:
            transforms_string, buttons_string = string.split("&")
        except ValueError:
            return None, None
        split_transform_strings = transforms_string.split("|")
        transforms = {}
        for pair_string in split_transform_strings:
            transform = np.empty((4, 4))
            pair = pair_string.split(":")
            if len(pair) != 2:
                continue
            left_right_char = pair[0]  # is r or l
            transform_string = pair[1]
            values = transform_string.split(" ")
            c = 0
            r = 0
            count = 0
            for value in values:
                if not value:
                    continue
                transform[r][c] = float(value)
                c += 1
                if c >= 4:
                    c = 0
                    r += 1
                count += 1
            if count == 16:
                transforms[left_right_char] = transform
        buttons = parse_buttons(buttons_string)
        return transforms, buttons

    def extract_data(self, line):
        output = ""
        if self.tag in line:
            try:
                output += line.split(self.tag + ": ")[1]
            except ValueError:
                pass
        return output

    def get_transformations_and_buttons(self):
        with self._lock:
            return self.last_transforms, self.last_buttons

    def get_valid_transforms_and_buttons(self):
        while True:
            transforms, buttons = self.get_transformations_and_buttons()

            # Check if 'l' and 'r' are in transforms, indicating valid data
            if "l" in transforms and "r" in transforms:
                # print("Valid transforms received.")
                return transforms, buttons

            # Optionally log or print when data isn't available
            print("Waiting for valid transforms...")

            # Wait a bit before trying again (to avoid busy loop)
            time.sleep(0.1)  # Sleep for 100ms before retrying

    def read_logcat_by_line(self, connection):
        file_obj = connection.socket.makefile()
        while self.running:
            try:
                line = file_obj.readline().strip()
                data = self.extract_data(line)
                if data:
                    transforms, buttons = OculusReader.process_data(data)
                    with self._lock:
                        self.last_transforms, self.last_buttons = transforms, buttons
                    if self.print_FPS:
                        self.fps_counter.getAndPrintFPS()
            except UnicodeDecodeError:
                pass
        file_obj.close()
        connection.close()


class Oculus_mobile(DeviceBase):
    """A joystick controller for sending SE(3) commands as delta poses and binary command (open/close).

    This class is designed to provide a joystick controller for a mobile bimanual manipulator.
    It uses the rail oculus reader to listen to joystick events and map them to robot's task-space commands.

    The command comprises of three parts:

    * delta pose for each arms: a 6D vector of (x, y, z, roll, pitch, yaw) in meters and radians.
    * gripper for each arms: a binary command to open or close the gripper.

    Key bindings:
        ============================== =================
        Description                    Joystick event
        ============================== =================
        Toggle gripperR (open/close)   RTr (index finger)
        Toggle gripperL (open/close)   LTr (index finger)
        Move right arm                 IK of right joystick SE(3) 4x4 matrix
        Move left arm                  IK of left joystick SE(3) 4x4 matrix
        ============================== =================
    """

    def __init__(
        self,
        pos_sensitivity: float = 0.01,
        rot_sensitivity: float = 0.01,
    ):
        """
        Args:
            pos_sensitivity: Magnitude of input position command scaling for arms.
            rot_sensitivity: Magnitude of input rotation command scaling for arms.
        """
        # sensitivities
        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        # initialize OculusReader for joystick input
        self.oculus_reader = OculusReader()

        # command buffers
        self._close_gripper_left = False
        self._close_gripper_right = False
        self._delta_pos_left = np.zeros(3)  # (x, y, z) for left arm
        self._delta_rot_left = np.zeros(3)  # (roll, pitch, yaw) for left arm
        self._delta_pos_right = np.zeros(3)  # (x, y, z) for right arm
        self._delta_rot_right = np.zeros(3)  # (roll, pitch, yaw) for right arm

        # arm
        self._last_transform_left = np.eye(4)
        self._last_transform_right = np.eye(4)

        # gripper
        self._prev_LTr_state = False
        self._prev_RTr_state = False

        # dictionary for additional callbacks
        self._additional_callbacks = dict()

    def __del__(self):
        """Release the keyboard interface."""
        self.oculus_reader.stop()

    def reset(self):
        """Reset all commands."""
        self._close_gripper_left = False
        self._close_gripper_right = False
        self._delta_pos_left = np.zeros(3)
        self._delta_rot_left = np.zeros(3)
        self._delta_pos_right = np.zeros(3)
        self._delta_rot_right = np.zeros(3)

    def __str__(self) -> str:
        """Returns: A string containing the information of joystick."""
        msg = f"Joystick Controller for SE(3): {self.__class__.__name__}\n"
        msg += "\t----------------------------------------------\n"

    def add_callback(self, key: str, func: Callable):
        """Add additional functions to bind keyboard.

        A list of available keys are present in the
        `carb documentation <https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/input-devices/keyboard.html>`__.

        Args:
            key: The keyboard button to check against.
            func: The function to call when key is pressed. The callback function should not
                take any arguments.
        """
        self._additional_callbacks[key] = func

    def advance(self) -> tuple[np.ndarray, bool, np.ndarray, bool, np.ndarray]:
        """
        Read joystick events and return command for Mobile Bimanual Manipulator.

        Returns:
            A tuple containing the delta pose commands for left arm, right arm.
        """
        # fetch latest controller data
        transforms, buttons = (
            self.oculus_reader.get_valid_transforms_and_buttons()
        )  # returns dict with 'leftJS', 'rightJS'

        # 1. grab current
        T_l = transforms["l"]
        T_r = transforms["r"]

        # 2. TRANSLATION DELTA: just position difference
        #    (new_pos - old_pos), then axis‐swap & sensitivity
        dp_l_raw = T_l[:3, 3] - self._last_transform_left[:3, 3]
        dp_r_raw = T_r[:3, 3] - self._last_transform_right[:3, 3]

        self._delta_pos_left = (
            np.array([-dp_l_raw[2], -dp_l_raw[0], dp_l_raw[1]]) * self.pos_sensitivity
        )
        self._delta_pos_right = (
            np.array([-dp_r_raw[2], -dp_r_raw[0], dp_r_raw[1]]) * self.pos_sensitivity
        )

        # 3. ROTATION DELTA: same as before, via delta‐matrix
        R_l_delta = T_l[:3, :3] @ self._last_transform_left[:3, :3].T
        R_r_delta = T_r[:3, :3] @ self._last_transform_right[:3, :3].T

        rv_l = Rotation.from_matrix(R_l_delta).as_rotvec() * self.rot_sensitivity
        rv_r = Rotation.from_matrix(R_r_delta).as_rotvec() * self.rot_sensitivity

        # re‐order [z, x, y] and flip signs on first two
        self._delta_rot_left = rv_l[[2, 0, 1]] * np.array([-1, -1, 1])
        self._delta_rot_right = rv_r[[2, 0, 1]] * np.array([-1, -1, 1])

        # 4. save current for next frame
        self._last_transform_left = T_l.copy()
        self._last_transform_right = T_r.copy()

        # 5. reset on button X
        if buttons.get("X", False):
            self.reset()

        # Gripper
        # Somewhere in your class, add:

        # Then in your update loop or callback:
        if buttons["LTr"] and not self._prev_LTr_state:
            self._close_gripper_left = not self._close_gripper_left

        # Update the previous state for the next cycle
        self._prev_LTr_state = buttons["LTr"]

        if buttons["RTr"] and not self._prev_RTr_state:
            self._close_gripper_right = not self._close_gripper_right
        # Update the previous state for the next cycle
        self._prev_RTr_state = buttons["RTr"]

        # return the commands
        return (
            np.concatenate([self._delta_pos_left, self._delta_rot_left]),  # Left arm
            self._close_gripper_left,  # Left gripper
            np.concatenate([self._delta_pos_right, self._delta_rot_right]),  # Right arm
            self._close_gripper_right,  # Right gripper
        )

    # Example:
    #  ({'l': array([[-0.224735 ,  0.415998 , -0.881158 , -0.0416255],
    #    [ 0.937851 , -0.153066 , -0.311457 , -0.103081 ],
    #    [-0.264441 , -0.89639  , -0.355745 ,  0.0854378],
    #    [ 0.       ,  0.       ,  0.       ,  1.       ]]),

    #    'r': array([[-0.753894 ,  0.601267 ,  0.264804 , -0.0686894],
    #    [-0.130283 ,  0.258232 , -0.957258 , -0.0819744],
    #    [-0.643948 , -0.756171 , -0.116345 ,  0.0297576],
    #    [ 0.       ,  0.       ,  0.       ,  1.       ]])}, {'A': False, 'B': False, 'RThU': True, 'RJ': False, 'RG': False, 'RTr': False, 'X': False, 'Y': False, 'LThU': True, 'LJ': False, 'LG': False, 'LTr': False, 'leftJS': (0.0, 0.0), 'leftTrig': (0.0,), 'leftGrip': (0.0,), 'rightJS': (0.0, 0.0), 'rightTrig': (0.0,), 'rightGrip': (0.0,)})
