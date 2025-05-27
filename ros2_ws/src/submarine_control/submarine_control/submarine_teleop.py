import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import select
import termios
import tty
import time

msg = """
Submarine Teleop
----------------
Controls:
    i/k: Forward/Backward Thrust
    t/b: Pitch Up/Down (works best with forward motion i)
    u/o: Turn Left/Right (works best with reverse motion k)
    SPACE: Reset all controls

w/x: Adjust thrust settings
e/c: Adjust fin settings

Max: Thrust {max_thrust_cmd:.1f}N, Fin {max_fin_angle:.2f}rad
Inc: Thrust {thrust_increment_step:.1f}N, Fin {fin_increment_step:.3f}rad

Current: Thrust {current_thrust_cmd:.1f}N, Turn {current_turn_fin_angle:.3f}rad, Pitch {current_pitch_fin_angle:.3f}rad

"""

moveBindings = {
    'i': (1.0, 0, 0),    # Increase forward thrust
    'k': (-1.0, 0, 0),   # Increase backward thrust
    'u': (0, 0, 1.0),    # Turn Left
    'o': (0, 0, -1.0),   # Turn Right
    't': (0, 1.0, 0),    # Pitch Up / Ascend
    'b': (0, -1.0, 0),   # Pitch Down / Descend
}

speedBindings = {
    'w': (1.1, 1.1),
    'x': (0.9, 0.9),
    'e': (1.1, 1.1),
    'c': (0.9, 0.9),
}

class SubmarineTeleop(Node):
    def __init__(self):
        super().__init__('submarine_teleop')
        
        self.MAX_FIN_ANGLE = 0.26
        self.MAX_FIN_INCREMENT = 0.1
        self.MAX_THRUST = 500.0
        self.MAX_THRUST_INCREMENT = 100.0
        
        self.thruster_pub = self.create_publisher(
            Float64, '/model/tethys/joint/propeller_joint/cmd_thrust', 10)
        self.vertical_fin_pub = self.create_publisher(
            Float64, '/model/tethys/joint/vertical_fins_joint/cmd_pos', 10)
        self.horizontal_fin_pub = self.create_publisher(
            Float64, '/model/tethys/joint/horizontal_fins_joint/cmd_pos', 10)
        
        self.max_thrust_cmd = 200.0
        
        self.current_thrust_cmd = 0.0
        self.current_pitch_fin_angle = 0.0
        self.current_turn_fin_angle = 0.0
        
        self.thrust_increment_step = 10.0
        self.fin_increment_step = 0.026

        self.thrust_decel_factor = 0.90
        self.fin_decel_factor = 0.90
        self.min_command_threshold = 0.001
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Submarine teleop initialized. Press Ctrl+C to exit.')
        
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def apply_deceleration(self):
        if abs(self.current_thrust_cmd) < self.min_command_threshold:
            self.current_thrust_cmd = 0.0
        else:
            self.current_thrust_cmd *= self.thrust_decel_factor
            
        if abs(self.current_pitch_fin_angle) < self.min_command_threshold:
            self.current_pitch_fin_angle = 0.0
        else:
            self.current_pitch_fin_angle *= self.fin_decel_factor
            
        if abs(self.current_turn_fin_angle) < self.min_command_threshold:
            self.current_turn_fin_angle = 0.0
        else:
            self.current_turn_fin_angle *= self.fin_decel_factor
    
    def reset_controls(self):
        self.current_thrust_cmd = 0.0
        self.current_pitch_fin_angle = 0.0
        self.current_turn_fin_angle = 0.0
        
    def publish_direct_commands(self):
        thrust_msg = Float64()
        thrust_msg.data = self.current_thrust_cmd
        self.thruster_pub.publish(thrust_msg)
        
        turn_fin_msg = Float64()
        turn_fin_msg.data = self.current_turn_fin_angle
        self.vertical_fin_pub.publish(turn_fin_msg)
        
        pitch_fin_msg = Float64()
        pitch_fin_msg.data = self.current_pitch_fin_angle
        self.horizontal_fin_pub.publish(pitch_fin_msg)
        
    def run(self):
        try:
            self.update_status_display()
            while rclpy.ok():
                key = self.getKey()
                
                if key == ' ':
                    self.reset_controls()
                
                elif key in moveBindings:
                    thrust_factor, pitch_factor, turn_factor = moveBindings[key]
                    
                    self.current_thrust_cmd += thrust_factor * self.thrust_increment_step
                    self.current_pitch_fin_angle += pitch_factor * self.fin_increment_step
                    self.current_turn_fin_angle += turn_factor * self.fin_increment_step
                    
                    self.current_thrust_cmd = max(min(self.current_thrust_cmd, self.max_thrust_cmd), -self.max_thrust_cmd)
                    self.current_pitch_fin_angle = max(min(self.current_pitch_fin_angle, self.MAX_FIN_ANGLE), -self.MAX_FIN_ANGLE)
                    self.current_turn_fin_angle = max(min(self.current_turn_fin_angle, self.MAX_FIN_ANGLE), -self.MAX_FIN_ANGLE)
                    
                elif key in speedBindings:
                    if key in ('w', 'x'):
                        max_thrust_factor, thrust_inc_factor = speedBindings[key]
                        new_max_thrust = self.max_thrust_cmd * max_thrust_factor
                        self.max_thrust_cmd = min(new_max_thrust, self.MAX_THRUST)
                        
                        new_thrust_increment = self.thrust_increment_step * thrust_inc_factor
                        self.thrust_increment_step = min(new_thrust_increment, self.MAX_THRUST_INCREMENT)
                    elif key in ('e', 'c'):
                        _, fin_inc_factor = speedBindings[key]
                        new_fin_increment = self.fin_increment_step * fin_inc_factor
                        self.fin_increment_step = min(new_fin_increment, self.MAX_FIN_INCREMENT)
                
                elif not key:
                    self.apply_deceleration()
                
                self.publish_direct_commands()
                self.update_status_display()
                
        except Exception as e:
            self.get_logger().error(f'Teleop error: {e}')
        finally:
            self.reset_controls()
            self.publish_direct_commands()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
    def update_status_display(self):
        print("\033c", end="")
        print(msg.format(
            max_thrust_cmd=self.max_thrust_cmd,
            max_fin_angle=self.MAX_FIN_ANGLE,
            thrust_increment_step=self.thrust_increment_step,
            fin_increment_step=self.fin_increment_step,
            current_thrust_cmd=self.current_thrust_cmd,
            current_turn_fin_angle=self.current_turn_fin_angle,
            current_pitch_fin_angle=self.current_pitch_fin_angle
        ))

def main(args=None):
    rclpy.init(args=args)
    teleop_node = SubmarineTeleop()
    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, teleop_node.settings)
        if rclpy.ok():
            teleop_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()