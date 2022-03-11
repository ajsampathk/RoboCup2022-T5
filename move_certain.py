#!/usr/bin/env python
from player import Player
import rospy
from multiprocessing import Process

rospy.init_node('Simple_controller')
Bot1 = Player("nao11")
# Bot2 = Player("nao12")

x = float(input('x='))

Bot1.move_x(x)
# p2 = Process(target=Bot2.move_x, args=(x,))

# p2.start()