# !/bin/usr/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

interval = 0.1

class calc:
    @staticmethod
    def vector_length(vector:Vector3)->float:
        """berechnet die Länge eines Vektors"""
        return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)
    @staticmethod
    def vector_dot(vector1:Vector3, vector2:Vector3):
        """berechnet das Skalarprodukt zweier Vektoren"""
        return vector1.x*vector2.x + vector1.y*vector2.y + vector1.z*vector2.z
    @staticmethod
    def vector_angle(vector1:Vector3, vector2:Vector3)->float:
        """berechnet den Winkel zwischen zwei Vektoren"""
        return math.acos(calc.vector_dot(vector1, vector2)/(calc.vector_length(vector1)*calc.vector_length(vector2)))

    @staticmethod
    def vector_length(vector:Vector3)->float:
        """berechnet die Länge eines Vektors"""
        return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)


class DriveToPointOdom(Node):
    def __init__(self, args=None):
        super().__init__('drive_to_point_odom')
        self._publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self._subscriber_odom = self.create_subscription(Odometry, 'odom', self._callback_odom, 10)
        self.ziel_position = Vector3()
        self.ziel_orientation = Vector3()
        self.destinations = []
        self.destinations.append([0.,1.,0.])
        self.drive_interval = interval

    def _callback_odom(self, msg):
        aktuelle_position = msg.twist.twist.linear
        aktuelle_orientation = msg.twist.twist.angular
        print(aktuelle_position)
        print(aktuelle_orientation)
        self._drive_to_next_point(aktuelle_position, aktuelle_orientation)

    def check_if_destination_reached(self, destination, )->bool:
        """überprüft, ob der Roboter das Ziel erreicht hat"""
        return False


    def drive_to_point(self, destination:Vector3, aktuelle_position:Vector3, aktuelle_orientation:Vector3, accuracy:float=0.1)->None:
        def _angle_to_turn(destination:Vector3, aktuelle_orientation:Vector3)->Vector3:
            """berechnet den Winkel, um den der Roboter sich drehen muss, um zum Ziel zu kommen"""
            angle = calc.vector_angle(destination, aktuelle_orientation)
            return Vector3(0,0,angle)


        def _distance_to_drive(destination:Vector3, aktuelle_position:Vector3)->Vector3:
            """berechnet die Distanz, die der Roboter zurücklegen muss, um zum Ziel zu kommen"""
            return Vector3(destination.x - aktuelle_position.x, destination.y - aktuelle_position.y, destination.z - aktuelle_position.z)

        angular = _angle_to_turn(destination, aktuelle_orientation)
        # if angular.z > max_angular:
        #     angular.z = max_angular
        #     self._publisher_cmd_vel.publish(Twist(angular=angular))
        # else: 
        linear = _distance_to_drive(destination, aktuelle_position)
        twist = Twist(linear=linear, angular=angular)
        self._publisher_cmd_vel.publish(twist)


    def _drive_to_next_point(self, aktuelle_position:Vector3, aktuelle_orientation:Vector3)->None:
        if len(self.destinations) > 0:
            self.drive_to_point(self.destinations[0], aktuelle_position, aktuelle_orientation)


          








def main(args=None)->None:
    rclpy.init(args=args)
    rclpy.spin(DriveToPointOdom())
    rclpy.shutdown()