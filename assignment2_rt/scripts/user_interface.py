import rclpy
import sys
import select
import termios
import tty
from rclpy.node import Node
from geometry_msgs.msg import Twist 
# Import custom services defined in the assignment2_custom_msgs package
from assignment2_custom_msgs.srv import GetAvgVel, ChangeThreshold

class UI(Node):

    def __init__(self):
        # Initialize the node with the name 'ui_node'
        super().__init__('ui_node')

        #  Publishers 
        # We publish to '/user_cmd' instead of directly to '/cmd_vel'.
        self.pub = self.create_publisher(Twist, '/user_cmd', 10)

        # Service Clients 
        # Create clients to communicate with the services provided by the robot_controller
        self.cli_avg = self.create_client(GetAvgVel, 'get_avg_vel')
        self.cli_thr = self.create_client(ChangeThreshold, 'change_threshold')
        
        # Internal Variables
        # Default starting speeds (stopped)
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        
        # Save the current terminal settings so we can restore them later.
        # This is needed because we change the terminal mode to read keys instantly.
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        # Set terminal to raw mode (read input immediately)
        tty.setraw(sys.stdin.fileno())
        # Check if there is input available 
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1) # Read one character
        else:
            key = '' # No key pressed
        # Restore original terminal settings (so 'Enter' works normally again)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def drive(self):
        print("\n--- MANUAL DRIVING MODE ---")
        print("Use keys to move:")
        print("   e : Forward")
        print("   d : Backward")
        print("   s : Turn Right")
        print("   f : Turn Left")
        print("   x : Stop Forcefully")
        print("   q : Quit to Main Menu")
        
        while True:
            # Read the latest key pressed by the user
            key = self.get_key()
            
            # 1. Update Target Speed based on Key
            if key == 'e':
                self.target_linear_vel = 0.5   # Set forward speed
                print("\rMoving Forward...    ", end="") # \r overwrites the line
            elif key == 'd':
                self.target_linear_vel = -0.5  # Set backward speed
                print("\rMoving Backward...   ", end="")
            elif key == 'f':
                self.target_angular_vel = 1.0  # Set left turn speed
                print("\rTurning Left...      ", end="")
            elif key == 's':
                self.target_angular_vel = -1.0 # Set right turn speed
                print("\rTurning Right...     ", end="")
            elif key == 'x':
                # Emergency Stop (Resets velocities to 0)
                self.target_linear_vel = 0.0
                self.target_angular_vel = 0.0
                print("\rSTOPPED.             ", end="")
            elif key == 'q':
                # Exit the driving loop and return to main menu
                print("\nExiting Drive Mode...")
                break 
            else:
                # If no key is pressed, keep the previous speed.
                pass

            # 2. Create and Publish Message 
            twist = Twist()
            twist.linear.x = float(self.target_linear_vel)
            twist.angular.z = float(self.target_angular_vel)
            
            # Send the command to the C++ node (Gatekeeper)
            self.pub.publish(twist)

    def get_avg(self):
        # Wait until the service is active (prevent errors)
        if not self.cli_avg.wait_for_service(1.0):
            print("Service not available!")
            return

        # Send request asynchronously
        future = self.cli_avg.call_async(GetAvgVel.Request())
        # Wait for the response
        rclpy.spin_until_future_complete(self, future)
        
        # Print the result
        res = future.result()
        print(f"\n--> Average Linear: {res.avg_linear:.2f}")
        print(f"--> Average Angular: {res.avg_angular:.2f}")

    def set_threshold(self):
        try:
            # Get user input for new distance
            val = float(input("Enter new Safety Threshold : "))
            
            if not self.cli_thr.wait_for_service(1.0):
                print("Service not available!")
                return
            
            # Prepare the request
            req = ChangeThreshold.Request()
            req.new_threshold = val
            
            # Send and wait for response
            future = self.cli_thr.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            print(f"--> Threshold updated to {val} successfully!")
            
        except ValueError:
            print("Invalid Number!")

def main(args=None):
    # Initialize ROS 2 communication
    rclpy.init(args=args)
    node = UI()

    # --- Main Menu Loop ---
    print("------------------------------")
    print(" Robot Controller UI")
    print("------------------------------")
    print("1. Drive Robot (WASD)")
    print("2. Get Average Velocity")
    print("3. Change Safety Threshold")
    print("0. Exit")

    while True:
        try:
            # Get user choice
            choice = input("\nChoose option: ")
            
            # Execute the corresponding function
            if choice == '1': node.drive()
            elif choice == '2': node.get_avg()
            elif choice == '3': node.set_threshold()
            elif choice == '0': break # Exit program
            else: print("Invalid choice")
            
        except KeyboardInterrupt:
            # Handle Ctrl+C gracefully
            break

    # Cleanup ROS resources
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
