#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from assignment2_custom_msgs.srv import GetAvgVel, ChangeThreshold

class UI(Node):
    def __init__(self):
        super().__init__('ui_node')
        # Setup (Short & Simple)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cli_avg = self.create_client(GetAvgVel, 'get_avg_vel')
        self.cli_thr = self.create_client(ChangeThreshold, 'change_threshold')

    def drive(self):
        try:
            # Same Text Output as before
            lin = float(input("Enter Linear Velocity: "))
            ang = float(input("Enter Angular Velocity: "))
            
            msg = Twist()
            msg.linear.x = lin; msg.angular.z = ang
            
            # Send 10 times (Spamming logic)
            for _ in range(10): self.pub.publish(msg)
            print(f"--> Sent: Linear Velocity ={lin}, Angular Velocity={ang}")
            
        except ValueError:
            print("Erreur: Invalid Number!")

    def get_avg(self):
        if not self.cli_avg.wait_for_service(1.0):
            print("Service not available!")
            return

        future = self.cli_avg.call_async(GetAvgVel.Request())
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        print(f"--> Average Linear: {res.avg_linear:.2f}")
        print(f"--> Average Angular: {res.avg_angular:.2f}")

    def set_threshold(self):
        try:
            # Input new threshold
            val = float(input("Enter new Safety Threshold (e.g. 0.5): "))
            # Call service to update
            if not self.cli_thr.wait_for_service(1.0):
                print("Service not available!")
                return
            # Prepare request
            req = ChangeThreshold.Request()
            req.new_threshold = val
            # Call service
            future = self.cli_thr.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            print(f"--> Threshold updated to {val} successfully!")
            # No error handling for simplicity
        except ValueError:
            print("Invalid Number!")

def main(args=None):
    rclpy.init(args=args)
    node = UI()

    # The nice Menu output you wanted
    print("\n ------------------------------")
    print("\n ROBOT CONTROLLER UI")
    print("\n ------------------------------")
    print("1. Drive Robot")
    print("2. Get Average Velocity ")
    print("3. Change Safety Threshold")
    print("0. Exit")

    while True:
        choice = input("\nChoose option : ")
        # Handle choices
        if choice == '1': node.drive()
        elif choice == '2': node.get_avg()
        elif choice == '3': node.set_threshold()
        elif choice == '0': break
        else: print("Invalid choice")

    rclpy.shutdown()

if __name__ == '__main__':
    main()