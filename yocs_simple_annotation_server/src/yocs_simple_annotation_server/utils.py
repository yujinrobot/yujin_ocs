#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
#

import yaml
import yocs_msgs.msg as yocs_msgs
import ar_track_alvar.msg as ar_msgs
from rospy_message_converter import message_converter
import genpy


def load_annotations_from_file(annotation, filename):
    yaml_data = None

    conversion_dict = {}
    conversion_dict['walls'] = yocs_msgs.Wall
    conversion_dict['columns'] = yocs_msgs.Column
    conversion_dict['tables'] = yocs_msgs.Table
    conversion_dict['ar_markers'] = ar_msgs.AlvarMarker

    with open(filename) as f:
        yaml_data = yaml.load(f)

        for key, val in yaml_data.items():
            annotation[key] = []
            for v in val:
                message = conversion_dict[key]()
                genpy.message.fill_message_args(message, v)
                annotation[key].append(message)


def write_annotations_into_file(annotation, filename):
    out = {}
    for key, val in annotation.items():
        out[key] = []
        for v in val:
            out[key].append(message_converter.convert_ros_message_to_dictionary(v))

    with open(filename, 'w') as f:
        f.write(yaml.dump(out))
