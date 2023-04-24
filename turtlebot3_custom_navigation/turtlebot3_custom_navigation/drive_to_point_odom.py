# !/bin/usr/env python3
import math
from time import sleep
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

interval = 1.0
max_linear_velocity = 0.2
max_angluar_velocity = 0.2

class calc:
    @staticmethod
    def vector_length(vector:Vector3)->float:
        """berechnet die Länge eines Vektors"""
        result = math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)
        #print(f"vektorlänge: {result}")
        return result
    @staticmethod
    def vector_dot(vector1:Vector3, vector2:Vector3)->float:
        """berechnet das Skalarprodukt zweier Vektoren"""
        result = vector1.x*vector2.x + vector1.y*vector2.y + vector1.z*vector2.z
        #print(f"vektoren skalarprodukt: {result}")
        return result
    @staticmethod
    def vector_angle(vector1:Vector3, vector2:Vector3)->float:
        """berechnet den Winkel zwischen zwei Vektoren"""
        #print("vektor_angle")
        #print("skalarprodukt: ", calc.vector_dot(vector1, vector2))
        #print("vektorlänge vektor a: ", calc.vector_length(vector1))
        #print("vektorlänge vektor b: ", calc.vector_length(vector2))
        zaehler = calc.vector_dot(vector1, vector2)
        # !!! WICHTIG Nenner kann null werden -> abfangen!!!
        nenner = (calc.vector_length(vector1)*calc.vector_length(vector2))
        if nenner < 0.0001:
            return 0.0
        result=  math.acos(zaehler/nenner)
        #print(f"vektoren winkel: {result}")
        return result

    @staticmethod
    def vector_length(vector:Vector3)->float:
        """berechnet die Länge eines Vektors"""
        return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)


class DriveToPointOdom(Node):
    def __init__(self, args=None):
        super().__init__('drive_to_point_odom')
        self._publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self._subscriber_odom = self.create_subscription(Odometry, 'odom', self._callback_odom, 10)
        self.position = Vector3()
        self.orientierung = Vector3()
        # self.ziel_position = Vector3()
        # self.ziel_orientation = Vector3()
        self.destinations = []
        # self.destinations.append(Vector3(x=2.0)) # deactivate
        DriveToPointOdom.load_targets()
        self.drive_interval = interval

    def _callback_odom(self, msg):
        aktuelle_position = msg.twist.twist.linear
        aktuelle_orientation = msg.twist.twist.angular
        print(f"aktuelle position: {aktuelle_position}\n") # deactivate
        # print(aktuelle_orientation) # deactivate
        self._drive_to_next_point(aktuelle_position, aktuelle_orientation)

    def check_if_destination_reached(self, destination:Vector3, aktuelle_position:Vector3,accuracy:float=0.2)->bool:
        """überprüft, ob der Roboter das Ziel erreicht hat"""
        # print("check_if_destination_reached")
        position_difference = Vector3(x=destination.x - aktuelle_position.x, y=destination.y - aktuelle_position.y, z=destination.z - aktuelle_position.z)
        # print(f"aktuelle position: {aktuelle_position}")
        # print(f"position_difference: {position_difference}")
        if calc.vector_length(position_difference) < accuracy:
            return True
        return False


    def drive_to_point(self, destination:Vector3, aktuelle_position:Vector3, aktuelle_orientation:Vector3, accuracy:float=0.1)->None:
        def _angle_to_turn(destination:Vector3, aktuelle_orientation:Vector3)->Vector3:
            """berechnet den Winkel, um den der Roboter sich drehen muss, um zum Ziel zu kommen"""
            #print("drive_to_point")
            #print(f"destination: {destination}")
            #print(f"aktuelle_orientation: {aktuelle_orientation}")
            angle = calc.vector_angle(destination, aktuelle_orientation)
            #print(f"angle: {angle}\n")
            return Vector3(z=angle)


        def _distance_to_drive(destination:Vector3, aktuelle_position:Vector3)->Vector3:
            """berechnet die Distanz, die der Roboter zurücklegen muss, um zum Ziel zu kommen"""
            x=destination.x - aktuelle_position.x
            y=destination.y - aktuelle_position.y
            z=destination.z - aktuelle_position.z
            length = abs(x)+abs(y)+abs(z)
            return Vector3(x=length)
        

        #prüfe Ziel erreicht
        if self.check_if_destination_reached(destination, aktuelle_position, accuracy):
            self.destinations.pop(0)
            self._publisher_cmd_vel.publish(Twist(linear=Vector3(), angular=Vector3())) # wenn Ziel erreicht, stoppe den Roboter



        #berechne Zielvektoren
        #print(f"destination: {destination}")
        #print(f"aktuelle_position: {aktuelle_position}")
        angular = _angle_to_turn(destination, aktuelle_orientation)
        #print(f"angular: {angular}\n")
        angular = Vector3()

        linear = _distance_to_drive(destination, aktuelle_position)
        #beschränke auf max winkelgeschwindigkeit
        if angular.z > max_angluar_velocity:
            angular.z = max_angluar_velocity
            #self._publisher_cmd_vel.publish(Twist(angular=angular))
        #else: 
        #beschränke auf max lineargeschwindigkeit
        if linear.x > max_linear_velocity:
            linear.x = max_linear_velocity

        #aktualisiere die gemerkte Position
        self.position.x += linear.x
        self.orientierung.z += angular.z
        twist = Twist(linear=linear, angular=angular)
        self._publisher_cmd_vel.publish(twist)

        #fahre 1 interval
        sleep(self.drive_interval) 


    def _drive_to_next_point(self, aktuelle_position:Vector3, aktuelle_orientation:Vector3)->None:
        if len(self.destinations) > 0:
            # print(self.destinations[0]) # deactivate
            self.drive_to_point(self.destinations[0], self.position, self.orientierung)


          

    @staticmethod
    def load_targets()->list(Vector3):
        """lädt die Ziele aus einer Datei"""
        destinations = []
        destination = Vector3()
        #beispiel
        destination.x = 0.0
        destination.y = 0.0
        destination.z = 0.0

        destinations.append(destination)







def main(args=None)->None:
    rclpy.init(args=args)
    rclpy.spin(DriveToPointOdom())
    rclpy.shutdown()

