# `umirobot-extras` Extra Content for the UMIRobot

| Module                                       | Description                                                                   | Author            | 
|----------------------------------------------|-------------------------------------------------------------------------------|-------------------|
| umirobot_control.commons                     | Module with shared content                                                    | Murilo M. Marinho |
| umirobot_control.configuration-space-control | Moving your UMIRobot in configuration space using a Python script             | Murilo M. Marinho |
| umirobot_control.task-space-control          | Moving your UMIRobot in task space using an constrained kinematic controller. | Murilo M. Marinho |


# Setup

```commandline
git clone https://github.com/mmmarinho/umirobot
cd umirobot
python3 -m venv venv
source venv/bin/activate
python -m pip install -r requirements.txt
```

# Quick Start

Each module has a `__main__.py` script. Run it in your favorite IDE (e.g. PyCharm) or through the commandline.

## Examples:

### Configuration-space control
```commandline
python -m umirobot_control.configuration_space_control
```

### Task-space control
```commandline
python -m umirobot_control.task_space_control
```