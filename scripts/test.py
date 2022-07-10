#! /usr/bin/env python
# lds_to_csv_node.py

import rospy
import csv
from sensor_msgs.msg import LaserScan

# Global vars
csv_writer = None

def scan_callback(msg):
    d = msg.ranges [90]
    if csv_writer is not None:
        csv_writer.writerow(d)
    print msg

def main():
    rospy.init_node("scan_to_csv")
    sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    fh = open("\\testread.csv", "wb")
    csv_writer = csv.writer(fh) # More constant params
    rospy.spin()
    fh.close()
    csv_writer = None

if __name__ == '__main__':
    main()
