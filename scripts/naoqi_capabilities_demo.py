#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from audio_common_msgs.msg import AudioInfo
from geometry_msgs.msg import Point, PoseStamped, Twist
from naoqi_utilities_msgs.msg import LedParameters, WordConfidence
from naoqi_utilities_msgs.srv import (
    ConfigureSpeech,
    Explore,
    GetInput,
    GetVolume,
    GoToPosture,
    MoveTo,
    PlayAnimation,
    PlaySound,
    PlayWebStream,
    PointAt,
    Say,
    SendURL,
    SetBreathing,
    SetMoveArmsEnabled,
    SetOpenCloseHand,
    SetSecurityDistance,
    SetStiffnesses,
    SetTrackerMode,
    SetVolume,
    ShowText,
)
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class NaoqiCapabilitiesDemo(Node):
    def __init__(self):
        super().__init__('naoqi_capabilities_demo')
        self.declare_parameter('robot_namespace', 'nao')
        self.declare_parameter('service_timeout_sec', 2.0)
        self.declare_parameter('step_pause_sec', 2.0)
        self.declare_parameter('section_pause_sec', 4.0)
        self.declare_parameter('interactive_demo', True)
        self.declare_parameter('call_tablet_services', False)
        self.declare_parameter('call_sound_playback', False)
        self.declare_parameter('call_exploration_services', False)
        self.namespace = self.get_parameter('robot_namespace').value.strip('/')
        self.service_timeout_sec = float(self.get_parameter('service_timeout_sec').value)
        self.step_pause_sec = float(self.get_parameter('step_pause_sec').value)
        self.section_pause_sec = float(self.get_parameter('section_pause_sec').value)
        self.interactive_demo = bool(self.get_parameter('interactive_demo').value)
        self.call_tablet_services = bool(self.get_parameter('call_tablet_services').value)
        self.call_sound_playback = bool(self.get_parameter('call_sound_playback').value)
        self.call_exploration_services = bool(self.get_parameter('call_exploration_services').value)

        self.cmd_vel_pub = self.create_publisher(Twist, self.topic('cmd_vel'), 10)
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, self.topic('joint_trajectory'), 10)
        self.speech_pub = self.create_publisher(String, self.topic('speech'), 10)
        self.led_pub = self.create_publisher(LedParameters, self.topic('set_leds'), 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, self.topic('goal_pose'), 10)

        self.create_subscription(Float32, self.topic('battery_percentage'), self.log_once('battery_percentage'), 10)
        self.create_subscription(JointState, self.topic('joint_states'), self.log_once('joint_states'), 10)
        self.create_subscription(Range, self.topic('sonar/left'), self.log_once('sonar/left'), 10)
        self.create_subscription(Range, self.topic('sonar/right'), self.log_once('sonar/right'), 10)
        self.create_subscription(AudioInfo, self.topic('mic_info'), self.log_once('mic_info'), 10)
        self.create_subscription(WordConfidence, self.topic('word_recognized'), self.log_once('word_recognized'), 10)

        self.logged_topics = set()
        self.has_run = False

    def topic(self, name):
        return f'/{self.namespace}/{name}' if self.namespace else f'/{name}'

    def service(self, name):
        return self.topic(name)

    def log_once(self, topic_name):
        def callback(_msg):
            if topic_name not in self.logged_topics:
                self.logged_topics.add(topic_name)
                self.get_logger().info(f'Received {self.topic(topic_name)}')
        return callback

    def wait_for_service(self, client):
        return client.wait_for_service(timeout_sec=self.service_timeout_sec)

    def section(self, title):
        self.get_logger().info('')
        self.get_logger().info('=' * 72)
        self.get_logger().info(f'DEMO SECTION: {title}')
        self.get_logger().info('=' * 72)
        self.pause(self.section_pause_sec, f'Starting {title}')

    def pause(self, seconds=None, reason='Next step'):
        duration = self.step_pause_sec if seconds is None else float(seconds)
        if duration <= 0.0:
            return
        whole_seconds = max(1, int(duration))
        for remaining in range(whole_seconds, 0, -1):
            self.get_logger().info(f'{reason} in {remaining}s')
            time.sleep(1.0)
        fractional = duration - whole_seconds
        if fractional > 0.0:
            time.sleep(fractional)

    def announce_step(self, title):
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'DEMO STEP: {title}')

    def choose_option(self, title, options, default_index=0, allow_skip=True):
        if not options:
            return None
        if not self.interactive_demo:
            return options[default_index]

        self.get_logger().info('')
        self.get_logger().info(f'SELECT: {title}')
        for index, (label, _) in enumerate(options, start=1):
            self.get_logger().info(f'  {index}. {label}')
        if allow_skip:
            self.get_logger().info('  s. Skip this capability')

        while rclpy.ok():
            try:
                answer = input(f'Choose option [default {default_index + 1}]: ').strip().lower()
            except EOFError:
                answer = ''

            if answer == '':
                return options[default_index]
            if allow_skip and answer in ('s', 'skip'):
                return None
            if answer.isdigit():
                selected = int(answer) - 1
                if 0 <= selected < len(options):
                    return options[selected]
            self.get_logger().warn('Invalid selection. Enter a number from the list, or s to skip.')

        return None

    def run_selected_action(self, title, options, default_index=0):
        choice = self.choose_option(title, options, default_index=default_index)
        if choice is None:
            self.get_logger().info(f'Skipping {title}')
            self.pause(reason=f'Continuing after skipped {title}')
            return None
        label, action = choice
        self.announce_step(f'{title}: {label}')
        result = action()
        self.pause(reason=f'Finished {title}')
        return result

    def choose_next_section(self, current_index):
        if not self.interactive_demo:
            return current_index + 1

        self.get_logger().info('')
        self.get_logger().info('SECTION COMPLETE')
        self.get_logger().info('  Enter / n: continue to next section')
        self.get_logger().info('  b: go back to previous section')
        self.get_logger().info('  q: quit demo')

        while rclpy.ok():
            try:
                answer = input('Next action [default n]: ').strip().lower()
            except EOFError:
                answer = ''

            if answer in ('', 'n', 'next'):
                return current_index + 1
            if answer in ('b', 'back'):
                return max(0, current_index - 1)
            if answer in ('q', 'quit', 'exit'):
                return None
            self.get_logger().warn('Invalid selection. Press Enter/n, b, or q.')

    def choose_next_capability(self, current_index):
        if not self.interactive_demo:
            return current_index + 1

        self.get_logger().info('')
        self.get_logger().info('CAPABILITY COMPLETE')
        self.get_logger().info('  Enter / n: continue to next capability')
        self.get_logger().info('  b: go back to previous capability')
        self.get_logger().info('  q: quit demo')

        while rclpy.ok():
            try:
                answer = input('Next capability [default n]: ').strip().lower()
            except EOFError:
                answer = ''

            if answer in ('', 'n', 'next'):
                return current_index + 1
            if answer in ('b', 'back'):
                return max(0, current_index - 1)
            if answer in ('q', 'quit', 'exit'):
                return None
            self.get_logger().warn('Invalid selection. Press Enter/n, b, or q.')

    def run_capability_steps(self, steps):
        index = 0
        while index is not None and index < len(steps) and rclpy.ok():
            label, action = steps[index]
            self.announce_step(label)
            action()
            index = self.choose_next_capability(index)
        return index is not None

    def call_service_choice(self, srv_type, name, title, options, default_index=0, required=False):
        choice = self.choose_option(title, options, default_index=default_index)
        if choice is None:
            self.get_logger().info(f'Skipping {title}')
            self.pause(reason=f'Continuing after skipped {name}')
            return None
        label, request_builder = choice
        return self.call_service(srv_type, name, request_builder, required=required, label=f'{title}: {label}')

    def call_service(self, srv_type, name, request_builder=None, required=False, label=None):
        self.announce_step(label or f'Call {self.service(name)}')
        client = self.create_client(srv_type, self.service(name))
        if not self.wait_for_service(client):
            level = self.get_logger().error if required else self.get_logger().warn
            level(f'Service unavailable, skipping: {self.service(name)}')
            self.destroy_client(client)
            self.pause(reason=f'Continuing after skipped {name}')
            return None

        request = srv_type.Request()
        if request_builder:
            request_builder(request)

        self.get_logger().info(f'Calling {self.service(name)}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            response = future.result()
            if hasattr(response, 'success'):
                self.get_logger().info(f'{name}: success={response.success} message={getattr(response, "message", "")}')
            else:
                self.get_logger().info(f'{name}: response={response}')
            self.destroy_client(client)
            self.pause(reason=f'Finished {name}')
            return response

        self.get_logger().warn(f'No response from {self.service(name)}')
        self.destroy_client(client)
        self.pause(reason=f'Continuing after {name}')
        return None

    def publish_driver_commands(self):
        return self.run_capability_steps([
            ('cmd_vel command', lambda: self.run_selected_action('cmd_vel command', [
                ('stop in place', lambda: self.cmd_vel_pub.publish(Twist())),
                ('hold small forward velocity until Enter', lambda: self.publish_twist_until_enter(0.05, 0.0, 0.0)),
                ('hold small rotation velocity until Enter', lambda: self.publish_twist_until_enter(0.0, 0.0, 0.15)),
            ])),
            ('joint trajectory command', lambda: self.run_selected_action('joint trajectory command', [
                ('neutral head', lambda: self.publish_head_trajectory(0.0, 0.0)),
                ('look left', lambda: self.publish_head_trajectory(0.4, 0.0)),
                ('look right', lambda: self.publish_head_trajectory(-0.4, 0.0)),
                ('look slightly down', lambda: self.publish_head_trajectory(0.0, 0.25)),
            ])),
            ('speech topic command', lambda: self.run_selected_action('speech topic command', [
                ('short greeting', lambda: self.publish_speech('Hello from the NAOqi bringup demo.')),
                ('capabilities message', lambda: self.publish_speech('I am testing my ROS capabilities.')),
                ('namespace message', lambda: self.publish_speech(f'My ROS namespace is {self.namespace or "root"}.')),
            ])),
            ('goal pose command', lambda: self.run_selected_action('goal pose command', [
                ('neutral base goal', lambda: self.publish_goal_pose(0.0, 0.0, 0.0)),
                ('small forward goal', lambda: self.publish_goal_pose(0.1, 0.0, 0.0)),
            ])),
        ])

    def publish_twist(self, linear_x, linear_y, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def publish_twist_until_enter(self, linear_x, linear_y, angular_z):
        self.publish_twist(linear_x, linear_y, angular_z)
        self.get_logger().info('Robot is moving. Press Enter to send cmd_vel 0,0,0 and stop.')
        if self.interactive_demo:
            try:
                input('Press Enter to stop: ')
            except EOFError:
                pass
        else:
            time.sleep(max(0.0, self.step_pause_sec))
        self.publish_twist(0.0, 0.0, 0.0)
        self.get_logger().info('Published stop command on cmd_vel.')

    def publish_head_trajectory(self, yaw, pitch):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['HeadYaw', 'HeadPitch']
        point = JointTrajectoryPoint()
        point.positions = [yaw, pitch]
        point.velocities = [0.1, 0.1]
        trajectory.points = [point]
        self.joint_trajectory_pub.publish(trajectory)

    def publish_speech(self, text_value):
        text = String()
        text.data = text_value
        self.speech_pub.publish(text)

    def publish_goal_pose(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = 'base_footprint'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = yaw
        goal.pose.orientation.w = 1.0
        self.goal_pose_pub.publish(goal)

    def publish_misc_commands(self):
        return self.run_selected_action('LED color command', [
            ('blue face LEDs', lambda: self.publish_led('FaceLeds', 0, 32, 255, 0.5)),
            ('green face LEDs', lambda: self.publish_led('FaceLeds', 0, 255, 64, 0.5)),
            ('white chest LED', lambda: self.publish_led('ChestLeds', 255, 255, 255, 0.5)),
        ])

    def publish_led(self, name, red, green, blue, duration):
        led = LedParameters()
        led.name = name
        led.red = red
        led.green = green
        led.blue = blue
        led.duration = duration
        self.led_pub.publish(led)

    def run_demo_once(self):
        if self.has_run:
            return
        self.has_run = True

        self.get_logger().info(f'Running demo against namespace: /{self.namespace}' if self.namespace else 'Running demo without namespace')
        self.pause(self.section_pause_sec, 'Demo begins')

        sections = [
            ('Generic driver topics', self.demo_driver_topics),
            ('Miscellaneous robot functions', self.demo_miscellaneous),
            ('Manipulation and body control', self.demo_manipulation),
            ('Navigation', self.demo_navigation),
            ('Perception', self.demo_perception),
            ('Speech and audio capabilities', self.demo_speech),
        ]
        if self.call_tablet_services:
            sections.append(('Tablet and interface capabilities', self.demo_tablet))
        sections.append(('Sensor callback observation window', self.demo_sensor_window))

        index = 0
        while index is not None and index < len(sections) and rclpy.ok():
            title, section_runner = sections[index]
            self.section(title)
            if section_runner() is False:
                self.get_logger().info('Demo stopped by operator.')
                rclpy.shutdown()
                return
            index = self.choose_next_section(index)

        if index is None:
            self.get_logger().info('Demo stopped by operator.')
            rclpy.shutdown()

    def demo_driver_topics(self):
        return self.publish_driver_commands()

    def demo_miscellaneous(self):
        return self.run_capability_steps([
            ('LED color command', self.publish_misc_commands),
            ('Autonomous blinking', lambda: self.call_service_choice(SetBool, 'toggle_blinking', 'Autonomous blinking', self.bool_options())),
            ('Basic awareness', lambda: self.call_service_choice(SetBool, 'toggle_awareness', 'Basic awareness', self.bool_options(default_false=True))),
            ('Autonomous life', lambda: self.call_service_choice(SetBool, 'set_autonomous_state', 'Autonomous life', self.bool_options(default_false=True))),
        ])

    def demo_manipulation(self):
        return self.run_capability_steps([
            ('Robot posture', lambda: self.call_service_choice(GoToPosture, 'go_to_posture', 'Robot posture', [
            ('Stand', lambda req: setattr(req, 'posture_name', 'Stand')),
            ('Sit', lambda req: setattr(req, 'posture_name', 'Sit')),
            ('Rest', lambda req: setattr(req, 'posture_name', 'Rest')),
        ])),
            ('Move arms during locomotion', lambda: self.call_service_choice(SetMoveArmsEnabled, 'set_move_arms_enabled', 'Move arms during locomotion', [
            ('enable both arms', lambda req: self.set_move_arms(req, True, True)),
            ('disable both arms', lambda req: self.set_move_arms(req, False, False)),
            ('left only', lambda req: self.set_move_arms(req, True, False)),
        ])),
            ('Hand open/close', lambda: self.call_service_choice(SetOpenCloseHand, 'set_open_close_hand', 'Hand open/close', [
            ('close left hand', lambda req: self.set_hand(req, 'LHand', False)),
            ('open left hand', lambda req: self.set_hand(req, 'LHand', True)),
            ('close right hand', lambda req: self.set_hand(req, 'RHand', False)),
            ('open right hand', lambda req: self.set_hand(req, 'RHand', True)),
        ])),
            ('Body breathing', lambda: self.call_service_choice(SetBreathing, 'toggle_breathing', 'Body breathing', [
            ('disable body breathing', lambda req: self.set_breathing(req, 'Body', False)),
            ('enable body breathing', lambda req: self.set_breathing(req, 'Body', True)),
            ('enable arms breathing', lambda req: self.set_breathing(req, 'Arms', True)),
        ])),
            ('Joint stiffness', lambda: self.call_service_choice(SetStiffnesses, 'set_stiffnesses', 'Joint stiffness', [
            ('head medium stiffness', lambda req: self.set_stiffnesses(req, ['HeadYaw', 'HeadPitch'], [0.5, 0.5])),
            ('head soft', lambda req: self.set_stiffnesses(req, ['HeadYaw', 'HeadPitch'], [0.1, 0.1])),
            ('body medium stiffness', lambda req: self.set_stiffnesses(req, ['Body'], [0.5])),
        ])),
            ('Tangential security distance', lambda: self.call_service_choice(SetSecurityDistance, 'set_tangential_security_distance', 'Tangential security distance', self.distance_options([0.1, 0.2, 0.3]))),
            ('Orthogonal security distance', lambda: self.call_service_choice(SetSecurityDistance, 'set_orthogonal_security_distance', 'Orthogonal security distance', self.distance_options([0.3, 0.4, 0.5]), default_index=1)),
            ('Arms collision protection', lambda: self.call_service_choice(SetBool, 'toggle_arms_collision_protection', 'Arms collision protection', self.bool_options())),
            ('Smart stiffness', lambda: self.call_service_choice(SetBool, 'toggle_smart_stiffness', 'Smart stiffness', self.bool_options())),
            ('Default motion security', lambda: self.call_service_choice(Trigger, 'enable_default_security', 'Default motion security', [('enable defaults', None)])),
            ('Animation', lambda: self.call_service_choice(PlayAnimation, 'play_animation', 'Animation', [
            ('greeting Hey_1', lambda req: setattr(req, 'animation_name', 'Stand/Gestures/Hey_1')),
            ('explain gesture', lambda req: setattr(req, 'animation_name', 'Stand/Gestures/Explain_1')),
            ('enthusiastic gesture', lambda req: setattr(req, 'animation_name', 'Stand/Gestures/Enthusiastic_4')),
        ])),
        ])

    def demo_navigation(self):
        steps = [
            ('Relative move command', lambda: self.call_service_choice(MoveTo, 'move_to', 'Relative move command', self.move_options())),
            ('Integrated navigation command', lambda: self.call_service_choice(MoveTo, 'navigate_to', 'Integrated navigation command', self.move_options())),
        ]
        if self.call_exploration_services:
            steps.extend([
                ('Start exploration radius', lambda: self.call_service_choice(Explore, 'start_exploring', 'Start exploration radius', [
                ('0.5 m radius', lambda req: setattr(req, 'radius', 0.5)),
                ('1.0 m radius', lambda req: setattr(req, 'radius', 1.0)),
            ])),
                ('Exploration visible window', lambda: self.pause(reason='Exploration visible window')),
                ('Stop exploration', lambda: self.call_service_choice(Trigger, 'stop_exploring', 'Stop exploration', [('stop now', None)])),
            ])
        return self.run_capability_steps(steps)

    def demo_perception(self):
        return self.run_capability_steps([
            ('Tracker mode', lambda: self.call_service_choice(SetTrackerMode, 'set_tracker_mode', 'Tracker mode', [
            ('stop tracker', lambda req: setattr(req, 'mode', 'stop')),
            ('start head tracking', lambda req: setattr(req, 'mode', 'start_head')),
            ('start body rotation tracking', lambda req: setattr(req, 'mode', 'start_body_rotation')),
        ])),
            ('Point at target', lambda: self.call_service_choice(PointAt, 'point_at', 'Point at target', [
            ('point forward with arms', lambda req: self.set_point_at(req, 'Arms', 1.0, 0.0, 0.0, 0.1)),
            ('point left with left arm', lambda req: self.set_point_at(req, 'LArm', 0.7, 0.3, 0.0, 0.1)),
            ('point right with right arm', lambda req: self.set_point_at(req, 'RArm', 0.7, -0.3, 0.0, 0.1)),
        ])),
        ])

    def demo_speech(self):
        steps = [
            ('Output volume', lambda: self.call_service_choice(SetVolume, 'set_volume', 'Output volume', [
            ('quiet volume 25', lambda req: setattr(req, 'volume', 25)),
            ('medium volume 50', lambda req: setattr(req, 'volume', 50)),
            ('loud volume 75', lambda req: setattr(req, 'volume', 75)),
        ], default_index=1)),
            ('Read output volume', lambda: self.call_service_choice(GetVolume, 'get_volume', 'Read output volume', [('read current volume', None)])),
            ('Speech recognition config', lambda: self.call_service_choice(ConfigureSpeech, 'configure_speech', 'Speech recognition config', [
            ('English hello vocabulary', lambda req: self.set_speech_config(req, ['hello', 'robot'], 'English', True)),
            ('Spanish demo vocabulary', lambda req: self.set_speech_config(req, ['hola', 'robot'], 'Spanish', True)),
            ('disable recognition expressions', lambda req: self.set_speech_config(req, [], '', False)),
        ])),
            ('Say text', lambda: self.call_service_choice(Say, 'say', 'Say text', [
            ('simple demo sentence', lambda req: self.set_say(req, 'This is a simple capability demo.', 'English', False, False)),
            ('animated greeting', lambda req: self.set_say(req, 'Hello, I am ready.', 'English', True, False)),
            ('Spanish greeting', lambda req: self.set_say(req, 'Hola, estoy listo.', 'Spanish', False, False)),
        ])),
            ('Stop current speech', lambda: self.call_service_choice(Trigger, 'shut_up', 'Stop current speech', [('stop speech', None)])),
            ('Stop all sounds', lambda: self.call_service_choice(Trigger, 'stop_all_sounds', 'Stop all sounds', [('stop all sounds', None)])),
        ]
        if self.call_sound_playback:
            steps.extend([
                ('Play local sound file', lambda: self.call_service_choice(PlaySound, 'play_sound', 'Play local sound file', [
                ('/home/nao/demo.wav', lambda req: setattr(req, 'file_path', '/home/nao/demo.wav')),
                ('/home/nao/audio/test.wav', lambda req: setattr(req, 'file_path', '/home/nao/audio/test.wav')),
            ])),
                ('Play web audio stream', lambda: self.call_service_choice(PlayWebStream, 'play_web_stream', 'Play web audio stream', [
                ('example wav stream', lambda req: setattr(req, 'url', 'http://example.com/audio.wav')),
            ])),
            ])
        return self.run_capability_steps(steps)

    def demo_tablet(self):
        return self.run_capability_steps([
            ('Show tablet text', lambda: self.call_service_choice(ShowText, 'show_text', 'Show tablet text', [
            ('English demo text', lambda req: self.set_show_text(req, 'NAOqi demo', 'English')),
            ('Spanish demo text', lambda req: self.set_show_text(req, 'Demo de NAOqi', 'Spanish')),
        ])),
            ('Show webview', lambda: self.call_service_choice(SendURL, 'show_webview', 'Show webview', [
            ('blank page', lambda req: setattr(req, 'url', 'about:blank')),
            ('local robot page', lambda req: setattr(req, 'url', 'http://198.18.0.1/apps/robot-page/index.html')),
        ])),
            ('Hide tablet', lambda: self.call_service_choice(Trigger, 'hide_tablet', 'Hide tablet', [('hide tablet', None)])),
            ('Request tablet input', lambda: self.call_service_choice(GetInput, 'get_input', 'Request tablet input', [
            ('text input prompt', lambda req: self.set_input_request(req, 'text', 'Demo input')),
            ('boolean prompt', lambda req: self.set_input_request(req, 'bool', 'Continue?')),
        ])),
            ('Take and show picture', lambda: self.call_service_choice(Trigger, 'take_and_show_picture', 'Take and show picture', [('take picture', None)])),
            ('App launcher', lambda: self.call_service_choice(SetBool, 'toggle_app_launcher', 'App launcher', self.bool_options(default_false=True))),
        ])

    def demo_sensor_window(self):
        self.get_logger().info('Demo finished. Listening briefly for sensor callbacks...')
        self.create_timer(5.0, self.shutdown)

    def bool_options(self, default_false=False):
        options = [
            ('enable / true', lambda req: setattr(req, 'data', True)),
            ('disable / false', lambda req: setattr(req, 'data', False)),
        ]
        return list(reversed(options)) if default_false else options

    def distance_options(self, distances):
        return [(f'{distance:.2f} m', lambda req, value=distance: setattr(req, 'distance', value)) for distance in distances]

    def move_options(self):
        return [
            ('zero move', lambda req: self.set_move(req, 0.0, 0.0, 0.0)),
            ('small forward move', lambda req: self.set_move(req, 0.1, 0.0, 0.0)),
            ('small rotation', lambda req: self.set_move(req, 0.0, 0.0, 0.2)),
        ]

    def set_move(self, request, x, y, theta):
        request.x_coordinate = x
        request.y_coordinate = y
        request.theta_coordinate = theta

    def set_move_arms(self, request, left_enabled, right_enabled):
        request.left_arm_enabled = left_enabled
        request.right_arm_enabled = right_enabled

    def set_hand(self, request, hand, state):
        request.hand = hand
        request.state = state

    def set_breathing(self, request, joint_group, enabled):
        request.joint_group = joint_group
        request.enabled = enabled

    def set_stiffnesses(self, request, joint_names, stiffnesses):
        request.joint_names = joint_names
        request.stiffnesses = stiffnesses

    def set_point_at(self, request, effector_name, x, y, z, speed):
        request.effector_name = effector_name
        request.point = Point(x=x, y=y, z=z)
        request.frame = request.FRAME_ROBOT
        request.speed = speed

    def set_speech_config(self, request, vocabulary, language, recognition_enabled):
        request.vocabulary = vocabulary
        request.language = language
        request.update_recognition_settings = True
        request.recognition_enabled = recognition_enabled
        request.audio_expression_enabled = False
        request.visual_expression_enabled = False

    def set_say(self, request, text, language, animated, asynchronous):
        request.text = text
        request.language = language
        request.animated = animated
        request.asynchronous = asynchronous

    def set_show_text(self, request, text, language):
        request.text = text
        request.language = language

    def set_input_request(self, request, input_type, content):
        request.type = input_type
        request.content = content

    def shutdown(self):
        self.get_logger().info('Shutting down demo node')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NaoqiCapabilitiesDemo()
    try:
        node.run_demo_once()
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
