import json
import math
import time
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image
from ollama import chat

MODEL = 'qwen3-vl:2b-instruct'
IMAGE_FILE = 'vlm_view.png'
STEP_DT = 1 / 240
END_EFFECTOR = 11
ARM_JOINTS = list(range(7))
FINGER_JOINTS = [9, 10]
HOME = [0, -0.5, 0, -2.0, 0, 1.6, 0.8]
TOOL_ORIENTATION = p.getQuaternionFromEuler([math.pi, 0, 0])


def simulate(steps):
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(STEP_DT)


def set_gripper(robot, opening):
    for joint in FINGER_JOINTS:
        p.setJointMotorControl2(
            robot, joint, p.POSITION_CONTROL,
            targetPosition=opening, force=60
        )
    simulate(120)


def move_to(robot, target, opening, steps=240):
    start = p.getLinkState(
        robot, END_EFFECTOR, computeForwardKinematics=True
    )[4]

    for step in range(1, steps + 1):
        alpha = step / steps
        position = [start[i] + alpha * (target[i] - start[i])
                    for i in range(3)]
        angles = p.calculateInverseKinematics(
            robot, END_EFFECTOR, position, TOOL_ORIENTATION,
            maxNumIterations=80
        )
        p.setJointMotorControlArray(
            robot, ARM_JOINTS, p.POSITION_CONTROL,
            targetPositions=angles[:7], forces=[200] * 7
        )
        for joint in FINGER_JOINTS:
            p.setJointMotorControl2(
                robot, joint, p.POSITION_CONTROL,
                targetPosition=opening, force=60
            )
        p.stepSimulation()
        time.sleep(STEP_DT)


def capture_image(filename):
    width, height = 640, 480
    view = p.computeViewMatrix(
        cameraEyePosition=[0.95, 0.0, 1.25],
        cameraTargetPosition=[0.48, 0, 0.05],
        cameraUpVector=[0, 1, 0]
    )
    projection = p.computeProjectionMatrixFOV(
        fov=55, aspect=width / height,
        nearVal=0.1, farVal=3.0
    )
    _, _, rgba, _, _ = p.getCameraImage(
        width, height, viewMatrix=view, projectionMatrix=projection,
        renderer=p.ER_TINY_RENDERER
    )
    rgb = np.asarray(rgba, dtype=np.uint8).reshape(height, width, 4)[:, :, :3]
    Image.fromarray(rgb).save(filename)
    print('Saved camera image:', Path(filename).resolve())


def choose_cube(instruction):
    schema = {
        'type': 'object',
        'properties': {
            'color': {'type': 'string', 'enum': ['red', 'green', 'blue', 'none']}
        },
        'required': ['color'],
        'additionalProperties': False
    }
    prompt = (
        'You are selecting one colored cube for a simulated robot arm. '
        'Inspect the attached camera image. The available cubes are red, green, and blue. '
        'Follow the user instruction, including spatial descriptions such as left, right, or middle. '
        'Respond with only JSON containing the color of the selected visible cube. '
        'Choose none if you cannot determine a single target. '
        f'User instruction: {instruction}'
    )
    print('Asking the local VLM to inspect the camera image...')
    response = chat(
        model=MODEL,
        messages=[{'role': 'user', 'content': prompt, 'images': [IMAGE_FILE]}],
        format=schema,
        options={'temperature': 0, 'num_ctx': 4096}
    )
    result = json.loads(response.message.content)
    color = result.get('color')
    if color not in ('red', 'green', 'blue'):
        raise ValueError('Model did not select one valid cube: ' + str(result))
    print('VLM selected:', color)
    return color


def assisted_pick_and_place(robot, cube, x, y):
    print('Moving above selected cube...')
    set_gripper(robot, 0.04)
    move_to(robot, [x, y, 0.35], 0.04)
    move_to(robot, [x, y, 0.13], 0.04)
    print('Closing gripper...')
    set_gripper(robot, 0.01)

    # Attach the cube for a reliable *assisted* grasp.
    ee_position, ee_orientation = p.getLinkState(
        robot, END_EFFECTOR, computeForwardKinematics=True
    )[4:6]
    cube_position, cube_orientation = p.getBasePositionAndOrientation(cube)
    inv_position, inv_orientation = p.invertTransform(
        ee_position, ee_orientation
    )
    relative_position, relative_orientation = p.multiplyTransforms(
        inv_position, inv_orientation, cube_position, cube_orientation
    )
    grasp = p.createConstraint(
        robot, END_EFFECTOR, cube, -1,
        p.JOINT_FIXED, [0, 0, 0],
        relative_position, [0, 0, 0],
        parentFrameOrientation=relative_orientation
    )

    print('Lifting and moving to destination...')
    move_to(robot, [x, y, 0.35], 0.01)
    move_to(robot, [0.35, -0.35, 0.35], 0.01)
    move_to(robot, [0.35, -0.35, 0.13], 0.01)
    print('Releasing selected cube...')
    p.removeConstraint(grasp)
    set_gripper(robot, 0.04)
    move_to(robot, [0.35, -0.35, 0.35], 0.04)
    simulate(240)
    print('Final selected cube position:', p.getBasePositionAndOrientation(cube)[0])


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(STEP_DT)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.loadURDF('plane.urdf')
    robot = p.loadURDF('franka_panda/panda.urdf', useFixedBase=True)
    p.resetDebugVisualizerCamera(1.5, 45, -40, [0.4, 0, 0.23])

    for joint, angle in enumerate(HOME):
        p.resetJointState(robot, joint, angle)
        p.setJointMotorControl2(
            robot, joint, p.POSITION_CONTROL,
            targetPosition=angle, force=200
        )
    for joint in FINGER_JOINTS:
        p.resetJointState(robot, joint, 0.04)
        p.setJointMotorControl2(
            robot, joint, p.POSITION_CONTROL,
            targetPosition=0.04, force=60
        )

    collision = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025]
    )
    cube_specs = {
        'red':   (-0.20, [1, 0.08, 0.08, 1]),
        'green': (0.00, [0.08, 0.95, 0.08, 1]),
        'blue':  (0.20, [0.08, 0.28, 1, 1]),
    }
    cubes = {}
    for color, (y, rgba) in cube_specs.items():
        visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.025] * 3, rgbaColor=rgba
        )
        cube_id = p.createMultiBody(
            baseMass=0.05,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=[0.55, y, 0.025]
        )
        cubes[color] = (cube_id, 0.55, y)

    simulate(120)
    capture_image(IMAGE_FILE)
    print('Look at vlm_view.png if you want to inspect the scene.')
    instruction = input('Your instruction (e.g. pick the red cube): ').strip()
    if not instruction:
        print('No instruction entered, so no movement will happen.')
        return
    color = choose_cube(instruction)
    cube_id, x, y = cubes[color]
    assisted_pick_and_place(robot, cube_id, x, y)
    capture_image('vlm_after.png')
    print('Finished. Final camera image: vlm_after.png')
    input('Press Enter to close PyBullet...')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('Stopped:', exc)
        print('If the model connection failed, check: sudo systemctl start ollama')
        raise
    finally:
        if p.isConnected():
            p.disconnect()
