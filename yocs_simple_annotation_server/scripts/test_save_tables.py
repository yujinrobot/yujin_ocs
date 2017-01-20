#!/usr/bin/env python
import rospy
from yocs_msgs.msg import TableList, Table
from yocs_msgs.srv import SaveTables

if __name__ == '__main__':
    rospy.init_node('test_save_tables')

    tlist = TableList()
    t1 = Table()
    t1.name = 'test_table'
    tlist.tables.append(t1)
    proxy = rospy.ServiceProxy('save_tables', SaveTables)
    proxy.wait_for_service()
    
    proxy([t1])
    rospy.loginfo("Saved")
    rospy.spin()
