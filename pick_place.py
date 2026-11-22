"""Stage 2: assisted pick-and-place with camera capture (no VLM)."""

import math
import time
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image

TIME_STEP = 1 / 240
EE = 11
ARM_JOINTS = list(range(7))
FINGER_JOINTS = [9, 10]
HOME = [0, -0.5, 0, -2.0, 0, 1.6, 0.8]
ORIENTATION = p.getQuaternionFromEuler([math.pi, 0, 0])
grip = 0.04


def simulate(steps=120):
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(TIME_STEP)


def set_gripper(robot, opening):
    global grip
    grip = opening
    for joint in FINGER_JOINTS:
        p.setJointMotorControl2(
            robot, joint, p.POSITION_CONTROL, targetPosition=grip, force=60
        )
    simulate(120)


def move_to(robot, target, steps=240):
    start = p.getLinkState(robot, EE, computeForwardKinematics=True)[4]
    for step in range(1, steps + 1):
        alpha = step / steps
        position = [start[i] + alpha * (target[i] - start[i]) for i in range(3)]
        angles = p.calculateInverseKinematics(
            robot, EE, position, ORIENTATION, maxNumIterations=80
        )
        p.setJointMotorControlArray(
            robot, ARM_JOINTS, p.POSITION_CONTROL,
            targetPositions=angles[:7], forces=[200] * 7
        )
        for joint in FINGER_JOINTS:
            p.setJointMotorControl2(
                robot, joint, p.POSITION_CONTROL,
                targetPosition=grip, force=60
            )
        p.stepSimulation()
        time.sleep(TIME_STEP)


def capture_camera(filename='robot_camera.png'):
    width, height = 640, 480
    view = p.computeViewMatrix(
        cameraEyePosition=[1.0, 0.8, 0.9],
        cameraTargetPosition=[0.4, -0.1, 0.15],
        cameraUpVector=[0, 0, 1]
    )
    projection = p.computeProjectionMatrixFOV(
        fov=60, aspect=width / height, nearVal=0.1, farVal=3.0
    )
    _, _, rgba, _, _ = p.getCameraImage(
        width, height, viewMatrix=view, projectionMatrix=projection,
        renderer=p.ER_TINY_RENDERER
    )
    pixels = np.asarray(rgba, dtype=np.uint8).reshape(height, width, 4)
    Image.fromarray(pixels).convert('RGB').save(filename)
    print('Camera image saved:', Path(filename).resolve())


def main():
    p.connect(p.GUI)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(TIME_STEP)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.loadURDF('plane.urdf')
        robot = p.loadURDF(
            'franka_panda/panda.urdf', basePosition=[0, 0, 0], useFixedBase=True
        )
        shape = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025]
        )
        visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025],
            rgbaColor=[1, 0, 0, 1]
        )
        cube = p.createMultiBody(
            baseMass=0.05, baseCollisionShapeIndex=shape,
            baseVisualShapeIndex=visual,
            basePosition=[0.55, 0, 0.025]
        )
        p.resetDebugVisualizerCamera(1.4, 45, -35, [0.3, 0, 0.25])
        for joint, angle in enumerate(HOME):
            p.resetJointState(robot, joint, angle)

        print('1. Opening gripper')
        set_gripper(robot, 0.04)
        print('2. Moving above cube')
        move_to(robot, [0.55, 0, 0.35])
        print('3. Lowering gripper')
        move_to(robot, [0.55, 0, 0.13])
        print('4. Closing gripper')
        set_gripper(robot, 0.01)

        # Assisted grasp: temporarily attach the cube to the end effector.
        ee_pos, ee_orn = p.getLinkState(
            robot, EE, computeForwardKinematics=True
        )[4:6]
        cube_pos, cube_orn = p.getBasePositionAndOrientation(cube)
        inv_pos, inv_orn = p.invertTransform(ee_pos, ee_orn)
        rel_pos, rel_orn = p.multiplyTransforms(
            inv_pos, inv_orn, cube_pos, cube_orn
        )
        grasp = p.createConstraint(
            robot, EE, cube, -1, p.JOINT_FIXED, [0, 0, 0],
            rel_pos, [0, 0, 0], parentFrameOrientation=rel_orn
        )

        print('5. Lifting cube')
        move_to(robot, [0.55, 0, 0.35])
        print('6. Moving to destination')
        move_to(robot, [0.35, -0.3, 0.35])
        print('7. Lowering cube')
        move_to(robot, [0.35, -0.3, 0.13])
        print('8. Releasing cube')
        p.removeConstraint(grasp)
        set_gripper(robot, 0.04)
        print('9. Raising arm')
        move_to(robot, [0.35, -0.3, 0.35])
        simulate(240)
        cube_pos, _ = p.getBasePositionAndOrientation(cube)
        print('Final cube position:', cube_pos)
        capture_camera('robot_camera.png')
        print('Simulation completed!')
        input('Press Enter to exit...')
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == '__main__':
    main()
