#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# UPDATED IMPORTS: Matching our assignment2_rtfinal package
from assignment2_rtfinal.srv import GetAverageVelocity, ChangeThreshold

class UI(Node):
    def __init__(self):
        super().__init__('ui_node')
        # Setup
        self.pub = self.create_publisher(Twist, '/cmd_vel_input', 10) # Send to input topic for safety check
        self.cli_avg = self.create_client(GetAverageVelocity, 'get_average_velocity')
        self.cli_thr = self.create_client(ChangeThreshold, 'change_threshold')

    def drive(self):
        """
        New logic: Controls the robot using W/A/S/D keys.
        """
        print("\n--- DRIVING MODE ---")
        print("Use keys: 'w'(Fwd), 's'(Bkwd), 'a'(Left), 'd'(Right), 'x'(Stop)")
        print("Press 'q' to return to Main Menu.")

        while True:
            # Get single character input
            cmd = input("Command > ").lower()
            
            twist = Twist()
            
            # W/A/S/D Logic
            if cmd == 'w':
                twist.linear.x = 1.0
                print("--> Moving Forward")
            elif cmd == 's':
                twist.linear.x = -1.0
                print("--> Moving Backward")
            elif cmd == 'a':
                twist.angular.z = 1.0
                print("--> Rotating Left")
            elif cmd == 'd':
                twist.angular.z = -1.0
                print("--> Rotating Right")
            elif cmd == 'x':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print("--> STOP")
            elif cmd == 'q':
                print("Exiting Drive Mode...")
                break # Return to main menu
            else:
                print("Invalid Key! Use w/a/s/d/x")
                continue

            # Publish the command
            self.pub.publish(twist)

    def get_avg(self):
        if not self.cli_avg.wait_for_service(1.0):
            print("Service not available!")
            return

        future = self.cli_avg.call_async(GetAverageVelocity.Request())
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        print(f"--> Average Linear: {res.avg_linear:.2f}")
        print(f"--> Average Angular: {res.avg_angular:.2f}")

    def set_threshold(self):
        try:
            val = float(input("Enter new Safety Threshold (e.g. 0.5): "))
            
            if not self.cli_thr.wait_for_service(1.0):
                print("Service not available!")
                return
            
            req = ChangeThreshold.Request()
            req.new_threshold = val
            
            future = self.cli_thr.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            
            print(f"--> Threshold updated to {val} successfully!")
            
        except ValueError:
            print("Invalid Number!")

def main(args=None):
    rclpy.init(args=args)
    node = UI()

    print("------------------------------")
    print(" Robot Controller UI")
    print("------------------------------")
    print("1. Drive Robot (WASD Mode)")
    print("2. Get Average Velocity ")
    print("3. Change Safety Threshold")
    print("0. Exit")

    while True:
        try:
            choice = input("\nChoose option : ")
            if choice == '1': node.drive()
            elif choice == '2': node.get_avg()
            elif choice == '3': node.set_threshold()
            elif choice == '0': break
            else: print("Invalid choice")
        except KeyboardInterrupt:
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
