import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Int8
import sys, select, termios, tty
import time

# Topics
SET_GIMBAL_ATTITUDE_TOPIC = "ZR30/set_gimbal_attitude"
SET_ZOOM_TOPIC = "ZR30/set_zoom_level"
SET_FOCUS_TOPIC = "ZR30/set_focus"

# Node settings
NODE_NAME = 'cam_teleop_node'

# Movement settings
ANGLE_STEP = 5.0  # degrees
ZOOM_STEP = 1.0

# Key mappings
key_bindings = {
    'w': ('pitch', ANGLE_STEP),
    's': ('pitch', -ANGLE_STEP),
    'a': ('yaw', -ANGLE_STEP),
    'd': ('yaw', ANGLE_STEP),
    'z': ('zoom', ZOOM_STEP),
    'x': ('zoom', -ZOOM_STEP),
    'f': ('focus', 1), # Far
    'g': ('focus', -1), # Close
    'h': ('focus', 0), # Hold
    'j': ('focus', 2), # Auto
    'c': 'center',
}

# Help message
msg = """
Reading from the keyboard and Publishing to Camera Topics!
---------------------------
Gimbal Movement:
   w
a  s  d

w/s : pitch up/down
a/d : yaw left/right

Zoom:
z/x : zoom in/out

Focus:
f/g : far/close focus
h   : hold focus
j   : auto focus

c   : center gimbal
q to quit
"""

class CameraTeleop(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.attitude_publisher = self.create_publisher(Vector3Stamped, SET_GIMBAL_ATTITUDE_TOPIC, 10)
        self.zoom_publisher = self.create_publisher(Float32, SET_ZOOM_TOPIC, 10)
        self.focus_publisher = self.create_publisher(Int8, SET_FOCUS_TOPIC, 10)

        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.current_zoom = 1.0
        # Debounce settings for attitude publishing: when multiple attitude keys are
        # pressed quickly, wait this delay and publish only the final accumulated target.
        self.att_debounce = 0.0  # seconds
        self._last_att_input = 0.0
        self._pending_attitude = False

    def get_key(self, settings):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(msg)
        while rclpy.ok():
            key = self.get_key(settings)
            if key in key_bindings:
                action = key_bindings[key]
                if action == 'center':
                    self.current_pitch = 0.0
                    self.current_yaw = 0.0
                    # Center is an immediate command
                    att_msg = Vector3Stamped()
                    att_msg.vector.y = self.current_pitch
                    att_msg.vector.z = self.current_yaw
                    self.attitude_publisher.publish(att_msg)
                    self._pending_attitude = False
                    print(f"Centering gimbal.")
                elif action[0] == 'pitch':
                    # Update target but debounce actual publish
                    self.current_pitch += action[1]
                    self._last_att_input = time.time()
                    self._pending_attitude = True
                    print(f"Target pitch: {self.current_pitch}, will publish after debounce {self.att_debounce} seconds")
                elif action[0] == 'yaw':
                    # Update target but debounce actual publish
                    self.current_yaw += action[1]
                    self._last_att_input = time.time()
                    self._pending_attitude = True
                    print(f"Target yaw: {self.current_yaw}, will publish after debounce {self.att_debounce} seconds")
                elif action[0] == 'zoom':
                    self.current_zoom += action[1]
                    self.current_zoom = max(1.0, min(30.0, self.current_zoom)) # Clamp between 1.0 and 30.0
                    zoom_msg = Float32()
                    zoom_msg.data = self.current_zoom
                    self.zoom_publisher.publish(zoom_msg)
                    print(f"Zoom: {self.current_zoom}")
                elif action[0] == 'focus':
                    focus_msg = Int8()
                    focus_msg.data = int(action[1])
                    self.focus_publisher.publish(focus_msg)
                    print(f"Focus command: {action[1]}")

            elif key == 'q':
                break

            # If there is a pending attitude change and the debounce time elapsed,
            # publish the final accumulated attitude and clear the pending flag.
            if self._pending_attitude and (time.time() - self._last_att_input) >= self.att_debounce:
                att_msg = Vector3Stamped()
                att_msg.vector.y = self.current_pitch
                att_msg.vector.z = self.current_yaw
                self.attitude_publisher.publish(att_msg)
                self._pending_attitude = False
                print(f"Published -> Pitch: {self.current_pitch}, Yaw: {self.current_yaw}")

        # Before exiting, if there's a pending attitude, publish it
        if self._pending_attitude:
            att_msg = Vector3Stamped()
            att_msg.vector.y = self.current_pitch
            att_msg.vector.z = self.current_yaw
            self.attitude_publisher.publish(att_msg)
            print(f"Published before exit -> Pitch: {self.current_pitch}, Yaw: {self.current_yaw}")

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = CameraTeleop()
    teleop_node.run()
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
