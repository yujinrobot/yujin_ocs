#!/usr/bin/env python
import rospy
from yocs_msgs.msg import TableList, Table
from yocs_msgs.srv import SaveTables

if __name__ == '__main__':
    rospy.init_node('table_publisher')

    tlist = TableList()
    t1 = Table()
    t1.name = 't123123123'
    tlist.tables.append(t1)
    proxy = rospy.ServiceProxy('save_tables', SaveTables)
    publisher = rospy.Publisher('tables', TableList,latch=True)
    proxy.wait_for_service()
    
    proxy([t1])
    publisher.publish(tlist)
    rospy.loginfo("Published")
    rospy.spin()
