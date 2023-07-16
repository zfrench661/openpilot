import cereal.messaging as messaging
from cereal import log

from tools.joystick.billy.moving import get_control_command

import random
import time

ACTION_TIMING = 0.6

def bodi_dance(pm):
  # turn left and right
  # move forward and backward
  # do a 360
  directions = [
    (0,1), (0,-1),
    (0,-1), (0,1),
    (0,1), (0,-1),
    (0,-1), (0,1),
    (1,0), (-1,0),
    (0,1), (0,-1),
    (0,-1), (0,1),
  ]
  movements = directions
  for d in movements:
    sleep_time = ACTION_TIMING

    cmd_msg = get_control_command(d[0], d[1])
    pm.send('testJoystick', cmd_msg)

    time.sleep(sleep_time)

  cmd_msg = get_control_command(0, 0)
  pm.send('testJoystick', cmd_msg)

