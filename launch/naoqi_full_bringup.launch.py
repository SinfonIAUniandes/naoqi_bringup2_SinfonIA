import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessStart, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    nao_ip_arg = DeclareLaunchArgument(
        "nao_ip", default_value="127.0.0.1", description="IP address of the robot"
    )
    nao_port_arg = DeclareLaunchArgument(
        "nao_port", default_value="9559", description="Port number of the robot"
    )

    nao_ip = LaunchConfiguration("nao_ip")
    nao_port = LaunchConfiguration("nao_port")

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
    naoqi_speech_node = Node(
        package="naoqi_speech",
        executable="naoqi_speech_node",
        name="naoqi_speech_node",
        output="screen",
        arguments=["--ip", nao_ip, "--port", nao_port],
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
                    LogInfo(msg="----------------------------------------------------------"),
                    LogInfo(msg="--- Robot system is ready and configured. ---"),
                    LogInfo(msg="----------------------------------------------------------"),
                ],
            )
        ),
    ]

    ld = LaunchDescription(
        [
            nao_ip_arg,
            nao_port_arg,
            naoqi_driver_launch,
            naoqi_manipulation_node,
            naoqi_miscellaneous_node,
            naoqi_navigation_node,
            naoqi_perception_node,
            naoqi_speech_node,
        ]
        + startup_sequence_events
    )

    return ld