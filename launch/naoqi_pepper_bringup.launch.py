import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
import socket

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # It doesn't actually send data — just triggers interface resolution
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def generate_launch_description():
    # Set environment variable for the boot config file
    set_boot_config_env = SetEnvironmentVariable(
        "NAOQI_DRIVER_BOOT_CONFIG_FILE", "boot_config_PEPPER.json"
    )

    # Declare launch arguments
    nao_ip_arg = DeclareLaunchArgument(
        "nao_ip", default_value="127.0.0.1", description="IP address of the robot"
    )
    nao_port_arg = DeclareLaunchArgument(
        "nao_port", default_value="9559", description="Port number of the robot"
    )
    launch_video_server_arg = DeclareLaunchArgument(
        "launch_video_server",
        default_value="true",
        description="Launch the web_video_server for tablet streaming",
    )

    nao_ip = LaunchConfiguration("nao_ip")
    nao_port = LaunchConfiguration("nao_port")
    launch_video_server = LaunchConfiguration("launch_video_server")

    # Get the host IP to be used by the video server
    host_ip = get_local_ip()

    # --- Launch naoqi_driver ---
    naoqi_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("naoqi_driver"),
                "launch",
                "naoqi_driver.launch.py",
            )
        ),
        launch_arguments={"nao_ip": nao_ip, "nao_port": nao_port}.items(),
    )

    # --- Node Definitions ---
    naoqi_manipulation_node = Node(
        package="naoqi_manipulation",
        executable="naoqi_manipulation_node",
        name="naoqi_manipulation_node",
        output="screen",
        arguments=["--ip", nao_ip, "--port", nao_port],
    )
    naoqi_miscellaneous_node = Node(
        package="naoqi_miscellaneous",
        executable="naoqi_miscellaneous_node",
        name="naoqi_miscellaneous_node",
        output="screen",
        arguments=["--ip", nao_ip, "--port", nao_port],
    )
    naoqi_navigation_node = Node(
        package="naoqi_navigation",
        executable="naoqi_navigation_node",
        name="naoqi_navigation_node",
        output="screen",
        arguments=["--ip", nao_ip, "--port", nao_port],
    )
    naoqi_perception_node = Node(
        package="naoqi_perception",
        executable="naoqi_perception_node",
        name="naoqi_perception_node",
        output="screen",
        arguments=["--ip", nao_ip, "--port", nao_port],
    )
    naoqi_pepper_speech_node = Node(
        package="naoqi_speech",
        executable="naoqi_speech_node",
        name="naoqi_speech_node",
        output="screen",
        arguments=["--ip", nao_ip, "--port", nao_port],
    )
    naoqi_interface_node = Node(
        package="naoqi_interface",
        executable="naoqi_interface_node",
        name="naoqi_interface_node",
        output="screen",
        arguments=["--ip", nao_ip, "--port", nao_port],
        parameters=[{"video_server_ip": host_ip}],
    )

    # --- Web Video Server Node ---
    web_video_server_node = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        condition=IfCondition(launch_video_server),
    )

    # --- Initialization Sequence with Event Handlers ---

    # 1. Enable autonomous blinking
    call_toggle_blinking = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call /naoqi_miscellaneous_node/toggle_blinking std_srvs/srv/SetBool '{data: true}'",
            ]
        ],
        shell=True,
    )

    # 2. Disable autonomous life
    call_set_autonomous_state = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call /naoqi_miscellaneous_node/set_autonomous_state std_srvs/srv/SetBool '{data: false}'",
            ]
        ],
        shell=True,
    )

    # 3. Set robot to 'Stand' posture
    call_go_to_posture = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call /naoqi_manipulation_node/go_to_posture naoqi_utilities_msgs/srv/GoToPosture '{posture_name: \"Stand\"}'",
            ]
        ],
        shell=True,
    )

    # 4. Disable basic awareness
    call_toggle_awareness = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call /naoqi_miscellaneous_node/toggle_awareness std_srvs/srv/SetBool '{data: false}'",
            ]
        ],
        shell=True,
    )

    # 5. Stop the tracker
    call_stop_tracker = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call /naoqi_perception_node/set_tracker_mode naoqi_utilities_msgs/srv/SetTrackerMode '{mode: \"stop\"}'",
            ]
        ],
        shell=True,
    )

    # 6. Stop the app launcher before showing an image
    call_stop_app_launcher = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call /naoqi_interface_node/toggle_app_launcher std_srvs/srv/SetBool '{data: false}'",
            ]
        ],
        shell=True,
    )

    # 7. Show initial image on tablet
    call_show_initial_image = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call /naoqi_interface_node/show_image naoqi_utilities_msgs/srv/SendURL \"{url: 'http://198.18.0.1/apps/robot-page/img/SinfonIA-Tablet.png'}\"",
            ]
        ],
        shell=True,
    )

    # Chaining the service calls
    startup_sequence_events = [
        # Start the sequence once the miscellaneous node is ready
        RegisterEventHandler(
            OnProcessStart(
                target_action=naoqi_miscellaneous_node,
                on_start=[
                    LogInfo(msg="Miscellaneous node started. Beginning setup sequence..."),
                    call_toggle_blinking,
                ],
            )
        ),
        # Each call is executed upon completion of the previous one
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=call_toggle_blinking,
                on_completion=[
                    LogInfo(msg="Autonomous blinking enabled."),
                    call_set_autonomous_state,
                ],
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=call_set_autonomous_state,
                on_completion=[
                    LogInfo(msg="Autonomous life disabled."),
                    call_go_to_posture,
                ],
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=call_go_to_posture,
                on_completion=[
                    LogInfo(msg="Robot in 'Stand' posture."),
                    call_toggle_awareness,
                ],
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=call_toggle_awareness,
                on_completion=[
                    LogInfo(msg="Basic awareness disabled."),
                    call_stop_tracker,
                ],
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=call_stop_tracker,
                on_completion=[
                    LogInfo(msg="Tracker stopped."),
                    call_stop_app_launcher,
                ],
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=call_stop_app_launcher,
                on_completion=[
                    LogInfo(msg="App launcher stopped."),
                    call_show_initial_image,
                ],
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=call_show_initial_image,
                on_completion=[
                    LogInfo(msg="Initial image shown on tablet."),
                    LogInfo(msg="----------------------------------------------------------"),
                    LogInfo(msg="--- Robot system is ready and configured. ---"),
                    LogInfo(msg="----------------------------------------------------------"),
                ],
            )
        ),
    ]

    ld = LaunchDescription(
        [
            set_boot_config_env,
            nao_ip_arg,
            nao_port_arg,
            launch_video_server_arg,
            naoqi_driver_launch,
            naoqi_manipulation_node,
            naoqi_miscellaneous_node,
            naoqi_navigation_node,
            naoqi_perception_node,
            naoqi_pepper_speech_node,
            naoqi_interface_node,
            web_video_server_node,
        ]
        + startup_sequence_events
    )

    return ld