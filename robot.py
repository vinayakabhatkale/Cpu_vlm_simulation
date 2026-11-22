"""Stage 1: basic Franka Panda joint and gripper motion in PyBullet."""

import math
import time

import pybullet as p
import pybullet_data


def main():
    p.connect(p.GUI)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1 / 240)

        p.loadURDF("plane.urdf")
        robot = p.loadURDF(
            "franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True
        )
        p.loadURDF("cube_small.urdf", basePosition=[0.6, 0.15, 0.025])
        p.resetDebugVisualizerCamera(1.5, 45, -30, [0.3, 0, 0.4])

        home = [0, -0.4, 0, -2.2, 0, 1.8, 0.8]
        print("Basic robot simulation started. Press Ctrl+C to stop.")
        step = 0
        while p.isConnected():
            t = step / 240
            targets = home.copy()
            targets[0] = 0.5 * math.sin(t)
            for joint in range(7):
                p.setJointMotorControl2(
                    robot, joint, p.POSITION_CONTROL,
                    targetPosition=targets[joint], force=120
                )
            opening = 0.02 + 0.02 * math.sin(t)
            for joint in (9, 10):
                p.setJointMotorControl2(
                    robot, joint, p.POSITION_CONTROL,
                    targetPosition=opening, force=40
                )
            p.stepSimulation()
            time.sleep(1 / 240)
            step += 1
    except KeyboardInterrupt:
        print("Simulation stopped.")
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
