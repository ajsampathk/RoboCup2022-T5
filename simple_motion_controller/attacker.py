import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Twist, Quaternion 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import atan2
import move_bot3


if __name__ == '__main__':
    
    move_bot3.robot("nao11")
    move_bot3.robot("nao15")

