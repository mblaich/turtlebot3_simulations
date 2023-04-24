#!/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(Node):
    # Modell Burger bis ~ 0.22 | Waffle bis ~ 0.26
    max_geschwindigkeit = 0.20 # m/s
    # Modell Burger bis ~ 2.84 | Waffle bis ~ 1.82 -> blockiert bei pi/2 (90°) drehung 
    max_drehgeschwindigkeit = math.pi/2 # rad/s --> 90° in Radiant = pi/2
    # delta_max_geschwindigkeit = max_drehgeschwindigkeit/max_geschwindigkeit
    delta_max_geschwindigkeit = 1.0
    def __init__(self, args=None):
        super().__init__('drive_square')
        self._publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self._timer_period = 10.0 # ändere alle 10 sekunden einen neuen bewegungs befehl
        self.create_timer(self._timer_period, self._timer_callback)
        self.cmd_counter = 0
        self.get_logger().info('DriveSquare node has been created')


    def _timer_callback(self):
        bewegung = Vector3()
        winkel = Vector3()


        # verhältnis gerade fahren bis 90 grad drehung
        # 3:1
        schritte_gerade = 3 #  * self.delta_max_geschwindigkeit
        schritte_drehung = 1 # * 1/self.delta_max_geschwindigkeit

        # geradeaus und dann 90 grad nach links ...
        if self.cmd_counter % schritte_gerade+schritte_drehung <schritte_gerade:
            bewegung.x = self.max_geschwindigkeit
            winkel.z = 0.0
        elif self.cmd_counter % schritte_gerade+schritte_drehung < schritte_gerade + schritte_drehung:
            bewegung.x = 0.0
            winkel.z = self.max_drehgeschwindigkeit / schritte_drehung /self._timer_period # 90 grad drehung in schritte_drehung vielen Timmerintervallen

        self.cmd_counter += 1
        bewegungsbefehl = Twist(linear=bewegung, angular=winkel)
        self._publisher_cmd_vel.publish(bewegungsbefehl)




def main(args=None)->None:
    rclpy.init(args=args)
    rclpy.spin(DriveSquare())
    rclpy.shutdown()