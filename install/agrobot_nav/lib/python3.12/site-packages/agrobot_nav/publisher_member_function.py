import rclpy
from rclpy.node import Node
from tutorial_interfaces.msg import Num

import sys
import termios
import tty
import threading

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)
        
        self.command = ""
        
        # Save terminal settings to restore them later
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Start a background thread to watch the keyboard
        # We use a thread so 'input()' doesn't block rclpy.spin()
        self.thread = threading.Thread(target=self.get_key_loop)
        self.thread.daemon = True
        self.thread.start()

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Terminal Listener Started. Use 'b' or '1' (Ctrl+C to quit)")

    def get_key_loop(self):
        """This runs in a background thread to capture keys from the terminal."""
        import time
        while True:
            try:
                tty.setcbreak(sys.stdin.fileno())
                # This reads exactly 1 character from the terminal
                key = sys.stdin.read(1)
                if not key:
                    # EOF reached, prevent tight loop
                    time.sleep(0.1)
                    continue
                self.command = key
            except Exception:
                # If stdin is not a tty or other error occurs, sleep to prevent tight loop
                time.sleep(0.1)

    def timer_callback(self):
        msg = Num()
        
        # Check the last key pressed
        if self.command == "w":
            msg.num = 1
        elif self.command == "a":
            msg.num = 2
        elif self.command == "s":
            msg.num = 3
        elif self.command == "d":
            msg.num = 4
        elif self.command == "q":
            msg.num = 0
            
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.num}" (Current command: {self.command})')
        
        # Reset command if you want it to only trigger ONCE per press
        # Or leave it if you want it to keep publishing the last key
        # self.command = "" 

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # CRITICAL: Restore terminal settings so your terminal isn't broken
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, minimal_publisher.settings)
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()