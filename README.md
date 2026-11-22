# CPU-Friendly Vision-Language-Guided Robot Arm

**Local Qwen3-VL + Ollama + PyBullet + Franka Panda**

A three-stage robotics demo that uses a pretrained vision-language model (VLM) to choose a colored cube in a simulated scene based on a natural-language instruction. A conventional PyBullet controller then moves a simulated Franka Panda arm to pick and place the selected cube.

**No model training or dedicated GPU is required.** CPU inference can be slow, and Ollama may use available acceleration automatically. Use `ollama ps` during inference to inspect the processor being used.

> **Scope:** This is a *vision-language-guided object-selection and assisted pick-and-place* prototype, **not** an end-to-end learned robot policy. The VLM chooses the target color from a rendered image; the controller reads the object's known simulation coordinates and uses a temporary fixed constraint for assisted grasping.

## Demo: VLM-Guided Robotic Manipulation

This demonstration shows a pretrained Qwen3-VL-2B-Instruct
model integrated with a Franka Panda robotic arm in PyBullet.

The system:
1. Captures an image of the simulated workspace.
2. Receives a natural-language instruction.
3. Uses Qwen3-VL to select the requested object.
4. Executes an assisted pick-and-place operation.
5. Displays the final object position.

**Example instruction:** `Pick the blue cube`

**VLM output:** `blue`

**Result:** The robotic arm moves the selected cube
to the designated drop-off location.

### Video Demonstration

https://github.com/user-attachments/assets/c0c38f29-57a9-432b-93e4-b3346198efc5

**Example instruction:** `pick the blue cube`

The model selects `blue`, then the arm moves the matching cube to the fixed drop-off area.

```text
Your instruction (e.g. pick the red cube): pick the blue cube
Asking the local VLM to inspect the camera image...
VLM selected: blue
Moving above selected cube...
Closing gripper...
Lifting and moving to destination...
Releasing selected cube...
Finished. Final camera image: vlm_after.png
```

For a video/screenshot of a real run, add media to `assets/` and link it here. Do not present images generated from other scenes as results of this script.

## Architecture

```text
Natural-language instruction + PyBullet camera image
                        |
                        v
       Qwen3-VL-2B-Instruct via Ollama
                        |
             selected cube color
                        |
                        v
   Known cube coordinates from the simulation
                        |
      PyBullet IK + joint/gripper controllers
                        |
            Assisted pick-and-place
                        |
             Final scene image
```

## Software requirements

- Linux with an X11/desktop display to open the PyBullet GUI (tested by the developer on Ubuntu)
- Python 3.11 (the demo was developed with Python 3.11)
- PyBullet 3.2.7
- Ollama with `qwen3-vl:2b-instruct` downloaded
- NumPy and Pillow

## Set up

1. Install [Python](https://www.python.org/) and [Ollama](https://ollama.com/download/linux) on your machine.
2. Download the pretrained model **once**:

   ```bash
   ollama pull qwen3-vl:2b-instruct
   ```

3. Create a virtual environment and install dependencies:

   ```bash
   python3.11 -m venv .venv
   source .venv/bin/activate
   python -m pip install -r requirements.txt
   ```

4. Make sure Ollama is running (on many Ubuntu installations, it runs as a system service; otherwise start it in another terminal with `ollama serve`). Verify with:

   ```bash
   ollama list
   ```

## Run: from basic control to VLM

All three scripts use the same PyBullet installation and simulated Panda model. Run them one at a time:

```bash
source .venv/bin/activate
python robot.py          # Stage 1: basic joint and gripper motion
python pick_place.py     # Stage 2: assisted pick/place and robot_camera.png
python vlm_robot.py      # Stage 3: vision-language-guided object selection
```

Stop the current GUI before starting the next stage. The model is only needed for `vlm_robot.py`.

When prompted, try:

```text
pick the red cube
pick the blue cube
pick the green cube
```

The script captures a scene image, calls the local Ollama model, validates a structured color selection, and uses PyBullet to perform an assisted pick-and-place. It writes:

- `vlm_view.png` — scene before manipulation
- `vlm_after.png` — scene after manipulation

These files are local outputs and excluded by `.gitignore`.

## How it works

1. PyBullet loads a Franka Panda robot and three colored cubes.
2. Its software renderer (`ER_TINY_RENDERER`) captures a 640×480 scene image.
3. Qwen3-VL receives the image and the instruction, and produces a JSON selection of `red`, `green`, or `blue` (or `none`).
4. The program maps the color to **known simulated coordinates** and plans Cartesian motion via inverse kinematics.
5. The fingers close, and a temporary constraint attaches the selected cube to the end effector for reliable **assisted** transport.
6. The cube is released at a fixed destination and the final camera image is saved.

The program is a proof of concept: no collision-aware motion planning, learned grasp selection, reliable grasp-force physics, or visual pose estimation is implemented yet.

## Suggested experiments / next steps

- Shuffle object positions and evaluate selection across repeated trials.
- Estimate target position using camera/depth observations rather than reading it directly from simulation state.
- Implement contact-based grasping without the temporary constraint.
- Report object-selection accuracy, placement success rate, and inference latency across multiple trials.
- Make the program configurable via CLI flags and add automated tests.

## Project structure

```text
.
├── robot.py             # Stage 1: basic arm and gripper motion
├── pick_place.py        # Stage 2: scripted assisted pick/place + camera
├── vlm_robot.py         # Stage 3: main VLM integration script
├── requirements.txt     # Python dependencies
├── .gitignore           # Excludes environments, model weights and outputs
├── assets/
│   └── README.md        # Instructions for screenshots/demo video
└── README.md            # This documentation
```

## Notes

- Models are downloaded and managed by Ollama. Model weights are **not** committed to this repository.
- Running the PyBullet GUI requires desktop graphics support, though no discrete GPU is required.
- Do not use the simulated controller directly with real hardware without additional safety controls.

## Attribution

Built with [PyBullet](https://pybullet.org/), [Ollama](https://ollama.com/), and the pretrained [Qwen3-VL](https://huggingface.co/Qwen/Qwen3-VL-2B-Instruct) model. Check upstream licenses and model terms for your intended use.
