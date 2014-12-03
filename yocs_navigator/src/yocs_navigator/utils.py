'''
 utils.py
 LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
'''

import math

def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def sign(angle):
    
    if angle > 0.0:
        return 1
    elif angle < 0.0:
        return -1
    else:
        return 0
