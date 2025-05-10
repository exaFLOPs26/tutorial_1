"""
Script to run teleoperation with anubis in Isaac Lab environments.
exaFLOPs
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="Teleoperation for Mobile Bimanual Manipulator Isaac Lab environments."
)
parser.add_argument(
    "--disable_fabric",
    action="store_true",
    default=False,
    help="Disable fabric and use USD I/O operations.",
)
parser.add_argument(
    "--num_envs", type=int, default=1, help="Number of environments to simulate."
)
parser.add_argument(
    "--teleop_device",
    type=str,
    default="oculus",
    help="Device for interacting with environment",
)
parser.add_argument(
    "--task", type=str, default="Cabinet-anubis-teleop-v0", help="Name of the task."
)
parser.add_argument(
    "--sensitivity", type=float, default=1.0, help="Sensitivity factor."
)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# parse the arguments

app_launcher_args = vars(args_cli)
if args_cli.teleop_device.lower() == "handtracking":
    app_launcher_args["experience"] = (
        f'{os.environ["ISAACLAB_PATH"]}/apps/isaaclab.python.xr.openxr.kit'
    )
# launch omniverse app
app_launcher = AppLauncher(app_launcher_args)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import torch
import ipdb
import omni.log


from isaaclab.devices import (
    Se3Gamepad,
    Se3Keyboard,
    Se3SpaceMouse,
    Oculus_mobile,
)
from isaaclab.envs import ViewerCfg
from isaaclab.envs.ui import ViewportCameraController
from isaaclab.managers import TerminationTermCfg as DoneTerm

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.manager_based.anubis_fixed.cabinet import mdp
from isaaclab_tasks.utils import parse_env_cfg

# For now, I am using delta control.


def pre_process_actions(
    delta_pose_L: torch.Tensor,
    gripper_command_L: bool,
    delta_pose_R: torch.Tensor,
    gripper_command_R: bool,
) -> torch.Tensor:
    """Pre-process actions for the environment."""
    # compute actions based on environment

    # resolve gripper command
    gripper_vel_L = torch.zeros(delta_pose_L.shape[0], 1, device=delta_pose_L.device)
    gripper_vel_L[:] = -1.0 if gripper_command_L else 1.0

    gripper_vel_R = torch.zeros(delta_pose_R.shape[0], 1, device=delta_pose_R.device)
    gripper_vel_R[:] = -1.0 if gripper_command_R else 1.0

    # Ensure gripper velocities and base poses have the correct shapes
    gripper_vel_L = gripper_vel_L.reshape(-1, 1)  # Shape: (batch_size, 1)
    gripper_vel_R = gripper_vel_R.reshape(-1, 1)  # Shape: (batch_size, 1)

    return torch.concat(
        [delta_pose_L, delta_pose_R, gripper_vel_L, gripper_vel_R],
        dim=1,
    )


def main():
    """Running keyboard teleoperation with Isaac Lab manipulation environment."""
    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )
    # modify configuration
    env_cfg.terminations.time_out = None
    if "Lift" in args_cli.task:
        # set the resampling time range to large number to avoid resampling
        env_cfg.commands.object_pose.resampling_time_range = (1.0e9, 1.0e9)
        # add termination condition for reaching the goal otherwise the environment won't reset
        env_cfg.terminations.object_reached_goal = DoneTerm(
            func=mdp.object_reached_goal
        )

    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    # check environment name (for reach , we don't allow the gripper)
    if "Reach" in args_cli.task:
        omni.log.warn(
            f"The environment '{args_cli.task}' does not support gripper control. The device command will be ignored."
        )

    # create controller
    if args_cli.teleop_device.lower() == "keyboard":
        teleop_interface = Se3Keyboard(
            pos_sensitivity=0.05 * args_cli.sensitivity,
            rot_sensitivity=0.05 * args_cli.sensitivity,
        )
    elif args_cli.teleop_device.lower() == "oculus":
        teleop_interface = Oculus_mobile(
            pos_sensitivity=1.0 * args_cli.sensitivity,
            rot_sensitivity=0.8 * args_cli.sensitivity,
        )
    elif args_cli.teleop_device.lower() == "spacemouse":
        teleop_interface = Se3SpaceMouse(
            pos_sensitivity=0.000001 * args_cli.sensitivity,
            rot_sensitivity=0.000001 * args_cli.sensitivity,
        )
    elif args_cli.teleop_device.lower() == "gamepad":
        teleop_interface = Se3Gamepad(
            pos_sensitivity=0.1 * args_cli.sensitivity,
            rot_sensitivity=0.1 * args_cli.sensitivity,
        )
    else:
        raise ValueError(
            f"Invalid device interface '{args_cli.teleop_device}'. Supported: 'keyboard', 'spacemouse''handtracking'."
        )

    # add teleoperation key for env reset
    should_reset_recording_instance = False

    def reset_recording_instance():
        nonlocal should_reset_recording_instance
        should_reset_recording_instance = True

    teleop_interface2 = Se3Keyboard(
        pos_sensitivity=0.005 * args_cli.sensitivity,
        rot_sensitivity=0.01 * args_cli.sensitivity,
    )
    # Reset
    teleop_interface2.add_callback("R", reset_recording_instance)

    # reset environment
    env.reset()
    teleop_interface.reset()

    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            pose_L, gripper_command_L, pose_R, gripper_command_R = (
                teleop_interface.advance()
            )
            pose_L = pose_L.astype("float32")
            pose_R = pose_R.astype("float32")
            # convert to torch
            pose_L = torch.tensor(pose_L, device=env.device).repeat(env.num_envs, 1)
            pose_R = torch.tensor(pose_R, device=env.device).repeat(env.num_envs, 1)

            # pre-process actions
            actions = pre_process_actions(
                pose_L, gripper_command_L, pose_R, gripper_command_R
            )
            # apply actions
            env.step(actions)

            if should_reset_recording_instance:
                env.reset()
                teleop_interface.reset()
                should_reset_recording_instance = False

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
