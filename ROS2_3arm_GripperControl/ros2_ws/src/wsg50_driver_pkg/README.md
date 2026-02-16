
#  WSG-50 Driver for ROS 2

This package provides a **ROS 2 driver for the WSG-50** two-finger gripper. It includes communication interfaces, services, configuration files, and a launch file to control the gripper over TCP/IP in a ROS 2 environment.

---

##  Package: `wsg50_driver_pkg`

### Features

- TCP/IP-based communication with the WSG-50 gripper
- ROS 2 launch integration
- ROS 2 services for:
  - Setting target width
  - Getting gripper status
  - Calibrating the gripper
  - Reconnecting on communication failure
- Parameter configuration via YAML
- Modular, testable Python-based architecture

---

##  Directory Structure

```
wsg50_driver_pkg/
├── launch/
│   └── gripper.launch.py
├── config/
│   └── gripper_params.yaml
├── srv/
│   ├── SetWidth.srv
│   ├── GetStatus.srv
│   ├── Calibrate.srv
│   └── RecoverConnection.srv
├── wsg50_driver/
│   ├── __init__.py
│   ├── gripper_node.py
│   ├── wsg_interface.py
│   ├── constants.py
│   ├── utils.py
│   ├── gripper_control.py
│   └── ...
├── setup.py
├── setup.cfg
├── package.xml
└── resource/
    └── wsg50_driver_pkg
```

---

##  Installation

1. Clone the repository inside your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repo_url> wsg50_driver_pkg
   ```

2. Install dependencies - ROS2 Humble, pytest


3. Build the workspace:
   ```bash
   ./gripper.sh

---

##  How to Run

Launch the gripper node:
```bash
ros2 launch wsg50_driver_pkg gripper.launch.py
```

Or run it directly:
```bash
ros2 run wsg50_driver_pkg gripper_node
```

---

##  Available ROS Services

- `/set_width` — Sets the target gripper width
- `/get_status` — Returns gripper status info
- `/calibrate` — Initiates calibration sequence
- `/recover_connection` — Attempts to reconnect to gripper

---

##  Run Tests

```bash
pytest test/test_gripper_node.py --cov=wsg50_driver_pkg
```

---

##  ROS 2 Gripper Width Control

Control the distance between the gripper fingers using a ROS 2 service client.

###  File
`wsg50_driver_pkg/wsg50_driver/gripper_control.py`

###  Run It
```bash
ros2 run wsg50_driver_pkg gripper_control 40.0
```

###  Description
- Uses the `SetWidth` service to command the gripper to a specific width (mm).
- Prints the result and response message from the gripper node.

---

##  Module Description

| Module              | Description                                                                 |
|---------------------|-----------------------------------------------------------------------------|
| `gripper_node.py`   | Main ROS 2 node handling all gripper services.                              |
| `wsg_interface.py`  | TCP communication and command logic for WSG-50.                             |
| `utils.py`          | Utility functions for parsing and communication.                            |
| `constants.py`      | Error code and gripper-specific constants.                                  |
| `gripper_control.py`| CLI-based ROS 2 client to command gripper width.                            |

---

### `srv/`  
Custom ROS 2 service definitions:
- `SetWidth.srv`
- `GetStatus.srv`
- `Calibrate.srv`
- `RecoverConnection.srv`

These are built using `rosidl_generate_interfaces` and used in the ROS node.

---

##  Testing

You can test individual services using:

```bash
ros2 service call /set_width wsg50_driver_pkg/srv/SetWidth "{target_width: 50.0}"
ros2 service call /get_status wsg50_driver_pkg/srv/GetStatus "{}"
ros2 service call /calibrate wsg50_driver_pkg/srv/Calibrate "{}"
ros2 service call /recover_connection wsg50_driver_pkg/srv/RecoverConnection "{}"
```

---

##  Maintainer

**Bhumika Shree**  
Email: bhumikashree96@gmail.com
