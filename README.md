# `umirobot-extras` 

Extra Content for the [UMIRobot](https://mmmarinho.github.io/UMIRobot/).

# Contents

| Module                                       | Description                                                                   | Author            | 
|----------------------------------------------|-------------------------------------------------------------------------------|-------------------|
| umirobot_control.commons                     | Module with shared content                                                    | Murilo M. Marinho |
| umirobot_control.configuration-space-control | Moving your UMIRobot in configuration space using a Python script             | Murilo M. Marinho |
| umirobot_control.task-space-control          | Moving your UMIRobot in task space using an constrained kinematic controller. | Murilo M. Marinho |


# Setup

```bash
git clone https://github.com/mmmarinho/umirobot
cd umirobot
python3 -m venv venv
source venv/bin/activate
python -m pip install -r requirements.txt
```

# Quick Start

Each module has a `__main__.py` script. Run it in your favorite IDE (e.g. PyCharm) or through the commandline.

### Configuration-space contro
1. Open the `scenes/umirobot.ttt` on CoppeliaSim
2. Press the :arrow_forward: to start the simulation.
3. Make sure your Arduino is corrected to the correct port specified in `umirobot_control/configuration_space_control/__main__.py`. You can check it using the Arduino IDE or UMIRobotGUI.
4. Run the following script:
```bash
python -m umirobot_control.configuration_space_control
```

### Task-space control
1. Open the `scenes/umirobot.ttt` on CoppeliaSim
2. Press the :arrow_forward: to start the simulation.
3. Run the following script:
```bash
python -m umirobot_control.task_space_control
```
