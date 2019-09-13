#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import curses

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('kill_switch', anonymous=True)
em_pub = rospy.Publisher('eStop', Int32, queue_size=10)

stdscr.refresh()

key = ''
running = -1 # -1 init , 0 stop, 1 engine brake, 2 normal
while key != ord('q'):
	key = stdscr.getch()
	stdscr.refresh()
	if key == curses.KEY_BACKSPACE:
		if running == 0:
			em_pub.publish(2)
			stdscr.addstr(5, 20, "Normal Operation :)")
			running = 2
		elif running == 2:
			em_pub.publish(1)
			stdscr.addstr(5, 20, "Engine Brake       ")
			running = 1
		else:
			em_pub.publish(0)
			stdscr.addstr(5, 20, "Full Stop!!!       ")
			running = 0

curses.endwin()
