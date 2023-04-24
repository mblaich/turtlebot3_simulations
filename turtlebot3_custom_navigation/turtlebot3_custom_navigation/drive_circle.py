#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3


class DriveCircle(Node):
    first=True
    def __init__(self, args=None):
        super().__init__('drive_circle')
        self._publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        # self._subscriber_odom = self.create_subscription(Twist, 'odom', self._callback_odom, 10)
        self._timer_period = 10.0
        self.create_timer(self._timer_period, self._timer_callback)
        self.get_logger().info('DriveCircle node has been created')


    def _timer_callback(self):
        bewegung = Vector3()
        winkel = Vector3()

        if self.first:
            # self.first=False
            # Modell Burger bis ~ 0.22 | Waffle bis ~ 0.26
            bewegung.x = 0.1
            # bewegung.y = 0.0
            # bewegung.z = 0.1

            # Modell Burger bis ~ 2.84 | Waffle bis ~ 1.82
            # winkel.x = 0.0
            # winkel.y = 0.0
            winkel.z = 0.1 # 
        
        else:
            bewegung.x = 0.0
            # bewegung.y = 0.0
            # bewegung.z = 0.0

            # winkel.x = 0.0
            # winkel.y = 0.0
            winkel.z = 0.0




        bewegungsbefehl = Twist(linear=bewegung, angular=winkel)
        self._publisher_cmd_vel.publish(bewegungsbefehl)
        from time import sleep
        sleep(5)



 

def main(args=None)->None:
    """TODO: Docstring for main."""
    rclpy.init(args=args)

    rclpy.spin(DriveCircle())

    rclpy.shutdown()
    pass
