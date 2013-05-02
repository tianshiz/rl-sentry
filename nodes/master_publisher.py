#!/usr/bin/env python
#control program that lets you start and stop the bot.
import rospy
from std_msgs.msg import String
import curses
stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)

stdscr.addstr(0,10," up to start, down to stop, q to quit")
stdscr.refresh()
pub = rospy.Publisher('start_master', String)
rospy.init_node('commander_node')
r = rospy.Rate(10) # 10hz
key = ''
while key != ord('q'):
   key = stdscr.getch()
   stdscr.addch(20,25,key)
   stdscr.refresh()	
   if key == curses.KEY_UP: 
       pub.publish("1")
   elif key == curses.KEY_DOWN: 
       pub.publish("0")
   r.sleep()

curses.endwin()