#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
#

import rospy
import ar_track_alvar.msg as ar_msgs
import yocs_msgs.srv as yocs_srvs
import yocs_msgs.msg as yocs_msgs

from .utils import load_annotations_from_file, write_annotations_into_file


class SimpleAnnotationServer(object):

    __slots__ = ['publisher', 'service', 'filename', 'annotation', 'filename']

    def __init__(self):
        pass

    def init(self):
        self.filename = rospy.get_param("~filename")

        if not self.filename:
            self.logerr('Annotation file is required')
            return False

        self.annotation = {}
        load_annotations_from_file(self.annotation, self.filename)
        self._init_publishers()
        self._init_services()
        return True

    def _init_publishers(self):
        self.publisher = {}
        self.publisher['tables'] = rospy.Publisher('tables', yocs_msgs.TableList, latch=True)
        self.publisher['ar_markers'] = rospy.Publisher('ar_markers', ar_msgs.AlvarMarkers, latch=True)
        self.publisher['columns'] = rospy.Publisher('columns', yocs_msgs.ColumnList, latch=True)
        self.publisher['walls'] = rospy.Publisher('walls', yocs_msgs.WallList, latch=True)

    def _init_services(self):
        self.service = {}
        self.service['tables'] = rospy.Service('save_tables', yocs_srvs.SaveTables, self._process_save_tables)
        self.service['ar_markers'] = rospy.Service('save_ar_markers', yocs_srvs.SaveARMarkers, self._process_save_ar_markers)
        self.service['columns'] = rospy.Service('save_columns', yocs_srvs.SaveColumns, self._process_save_columns)
        self.service['walls'] = rospy.Service('save_walls', yocs_srvs.SaveWalls, self._process_save_walls)

    def _process_save_tables(self, req):
        self.annotation['tables'] = req.data
        self.update()
        return yocs_srvs.SaveTablesResponse(True)

    def _process_save_columns(self, req):
        self.annotation['columns'] = req.data
        self.update()
        return yocs_srvs.SaveColumnsResponse(True)

    def _process_save_ar_markers(self, req):
        self.annotation['ar_markers'] = req.data
        self.update()
        return yocs_srvs.SaveARMarkersResponse(True)

    def _process_save_walls(self, req):
        self.annotation['walls'] = req.data
        self.update()
        return yocs_srvs.SaveWallsResponse(True)

    def publish_annotations(self):
        # Tables
        message = yocs_msgs.TableList()
        message.tables = self.annotation['tables']
        self.publisher['tables'].publish(message)

        # AR Markers
        message = ar_msgs.AlvarMarkers()
        message.markers = self.annotation['ar_markers']
        self.publisher['ar_markers'].publish(message)

        # Columns
        message = yocs_msgs.ColumnList()
        message.obstacles = self.annotation['columns']
        self.publisher['columns'].publish(message)

        # Walls
        message = yocs_msgs.WallList()
        message.obstacles = self.annotation['walls']
        self.publisher['walls'].publish(message)

    def update(self):
        self.publish_annotations()
        write_annotations_into_file(self.annotation, self.filename)
        self.loginfo('Annotations have been updated.')

        for key in ['tables', 'columns', 'walls', 'ar_markers']:
            self.loginfo('  %s\t: %s' % (str(key), str(len(self.annotation[key]))))

    def spin(self):
        self.publish_annotations()
        rospy.spin()

    def loginfo(self, msg):
        rospy.loginfo('Annotation Server : ' + str(msg))

    def logerr(self, msg):
        rospy.logerr('Annotation Server : ' + str(msg))
