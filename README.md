# NAOqi Bringup for ROS 2 (SinfonIA)

This package, `naoqi_bringup2_sinfonIA`, is a ROS 2 metapackage designed to simplify the process of launching a complete, feature-rich interface for SoftBank Robotics' NAO and Pepper robots. It serves as a top-level entry point that aggregates and launches all the necessary nodes for robot interaction, from core drivers to high-level capabilities like manipulation, perception, and speech.

## Architecture

This is a metapackage that does not contain any nodes itself. Instead, its primary role is to declare dependencies on other packages and provide a main launch file that brings up the entire robot system in a structured and reliable way.

The system is composed of the following ROS 2 packages:

*   **`naoqi_driver`**: The core bridge that enables communication between the ROS 2 ecosystem and the robot's native NAOqi OS.
*   **`naoqi_manipulation`**: Manages robot posture (`Stand`, `Sit`, etc.), joint stiffness, hand control, and animations.
*   **`naoqi_miscellaneous`**: Controls the robot's autonomous behaviors (Autonomous Life, Basic Awareness, Blinking) and monitors battery status.
*   **`naoqi_navigation`**: Handles robot base movement, including relative moves and map-based navigation.
*   **`naoqi_perception`**: Controls perception-based actions like pointing at objects and tracking faces.
*   **`naoqi_speech`**: Manages all audio functionalities, including text-to-speech (TTS), speech recognition, and sound playback.
    > **Note**: For the Pepper robot (NAOqi 2.5), a specific node (`naoqi_pepper_speech_node`) is used as a workaround for a compatibility issue between the newer Python libraries and the older `ALMemory` event system. This workaround also means that this node handles touch sensor events (head, hands, bumpers) for Pepper, which are typically managed by `naoqi_driver` on NAO.
*   **`naoqi_interface`**: Handles interaction with the robot's tablet, such as displaying images, web pages, and video streams.
*   **`naoqi_bridge_msgs` & `naoqi_utilities_msgs`**: Provide the custom ROS 2 message and service definitions required for communication across all packages.

## Installation

To use this package, you must have a ROS 2 workspace containing this package and all of its dependencies.

1.  **Install `vcs-tool`** (if you don't have it):
    ```bash
    sudo apt-get update && sudo apt-get install python3-vcstool
    sudo apt-get install ros-jazzy-image-transport-plugins
    ```

2.  **Create and navigate to your ROS 2 workspace:**
    ```bash
    mkdir -p ~/ros2_naoqi_ws/src
    cd ~/ros2_naoqi_ws
    ```

3.  **Download the `.repos` file:**
    You need to get the `sinfonia.repos` file that lists all project repositories. For example, using `wget`:
    ```bash
    wget https://raw.githubusercontent.com/SinfonIAUniandes/naoqi_bringup2_SinfonIA/refs/heads/main/sinfonia.repos
    ```

4.  **Clone all repositories:**
    Navigate into your `src` folder and use `vcs` to import all the repositories listed in the file.
    ```bash
    cd src
    vcs import < ../sinfonia.repos
    ```

5.  **Install ROS 2 dependencies:**
    Go back to the root of your workspace and run `rosdep` to install any missing system dependencies.
    ```bash
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    ```

6.  **Install Python Dependencies:**
    Install the required Python packages using `pip`.
    ```bash
    pip install -r src/naoqi_bringup2_sinfonIA/requirements.txt
    ```

7.  **Build the workspace:**
    Compile all the packages using `colcon`
    ```bash
    cd ~/ros2_naoqi_ws
    colcon build --symlink-install
    ```

8.  **Source the workspace:**
    Before running the launch file, source your workspace's setup file.
    ```bash
    source install/setup.bash
    ```

## Usage

The primary way to use this package is through the provided master launch files. These files start all the necessary nodes and run a startup sequence to place the robot in a safe, ready-to-operate state.

### Launch Files and Robot Configuration

This package includes two main launch files, one for each robot type:

*   **`naoqi_full_bringup.launch.py`**: The launch file for the **NAO** robot.
*   **`naoqi_pepper_bringup.launch.py`**: The launch file for the **Pepper** robot.

A key part of the launch process is the driver configuration file in this package's `config/` directory. By default, `naoqi_full_bringup.launch.py` loads `config/naoqi_driver_nao.yaml`, and `naoqi_pepper_bringup.launch.py` loads `config/naoqi_driver_pepper.yaml`. These files contain the robot connection defaults, namespace, command topic remaps, generic driver stream enable flags, rates, and topic names without recompiling the driver.

The YAML files also select the matching `naoqi_driver` boot preset (`boot_config_NAO.json` or `boot_config_PEPPER.json`) for low-level robot defaults. They are the intended place to tune ROS-facing behavior for deployments.

### Launch Command

To launch the entire system, choose the appropriate launch file for your robot and run the corresponding command, replacing `<your_robot_ip>` with your robot's actual IP address.

**For NAO:**
```bash
ros2 launch naoqi_bringup2_sinfonIA naoqi_full_bringup.launch.py nao_ip:=<your_robot_ip>
```

**For Pepper:**
```bash
ros2 launch naoqi_bringup2_sinfonIA naoqi_pepper_bringup.launch.py nao_ip:=<your_robot_ip>
```

You can also specify the port if it's different from the default (`9559`):
```bash
ros2 launch naoqi_bringup2_sinfonIA <launch_file> nao_ip:=<your_robot_ip> nao_port:=<port>
```

To use a different driver configuration file:
```bash
ros2 launch naoqi_bringup2_sinfonIA naoqi_pepper_bringup.launch.py \
    nao_ip:=<your_robot_ip> \
    driver_params_file:=/absolute/path/to/naoqi_driver_params.yaml
```

Command input topics are remapped from launch arguments, for example:
```bash
ros2 launch naoqi_bringup2_sinfonIA naoqi_pepper_bringup.launch.py \
    nao_ip:=<your_robot_ip> \
    driver_cmd_vel_topic:=mobile_base/cmd_vel
```

### Recording Microphone Audio

The driver publishes microphone audio using `audio_common_msgs` on the topics configured in the bringup YAML file. By default, both robot configs publish raw audio on `/mic`, stamped audio on `/mic_stamped`, and stream metadata on `/mic_info`.

The default audio config is 16 kHz mono, 16-bit signed little-endian (`S16LE`). To write `/mic` to a WAV file with `audio_common`:

```bash
ros2 run audio_play audio_play_node --ros-args \
    -r audio:=/mic \
    -p dst:="$PWD/nao_recording.wav" \
    -p format:=wave \
    -p channels:=1 \
    -p sample_rate:=16000 \
    -p depth:=16 \
    -p sample_format:=S16LE
```

To change the microphone format, edit `converters.audio.*` in the selected config file under `config/`. For example, `converters.audio.sample_rate`, `converters.audio.channel_config`, `converters.audio.channels`, and `converters.audio.sample_format` should match the values passed to `audio_play_node`.

### Startup Sequence

Upon launching, the system performs a specific sequence of actions to initialize the robot correctly. This is handled robustly using event handlers to ensure each step completes before the next begins.

1.  **Nodes Start**: All the `naoqi` utility nodes are launched.
2.  **Blinking Enabled**: Autonomous blinking is enabled for a more natural appearance.
3.  **Autonomous Life Disabled**: The robot's default autonomous behaviors are disabled to give full control to the ROS 2 system.
4.  **Robot Stands Up**: The robot is commanded to go to the `Stand` posture.
5.  **Basic Awareness Disabled**: The robot's basic awareness (e.g., turning its head towards sounds) is disabled.
6.  **Tracker Stopped**: Any active tracking is stopped to ensure the robot is in a neutral state.
7.  **(Pepper Only)** **Initial Image Displayed**: An initial image is shown on the robot's tablet.

After this sequence, you will see a confirmation message in the console, and the robot will be ready to receive commands via the various ROS 2 topics and services.
