import json
import time

import cereal.messaging as messaging
from cereal import log
from cereal.visionipc import VisionIpcClient, VisionStreamType

IMAGE_HEIGHT, IMAGE_WIDTH = 416, 640
POS_EPS = 0.1
PERSON_WIDTH_TARGET = 0.4
PERSON_HEIGHT_TARGET = 0.55
IDLE_CNT_TARGET = 20
YOLO_MIN_CONF = 0.7

def get_control_command(x, y):
    x = max(-1.0, min(1.0, x))
    y = max(-1.0, min(1.0, y))

    dat = messaging.new_message('testJoystick')
    dat.testJoystick.axes = [x, y]
    dat.testJoystick.buttons = [False]

    return dat

def find_and_follow_human():
    pm = messaging.PubMaster(['testJoystick'])#, 'bodyFace'])
    sm = messaging.SubMaster(['logMessage'])

    follow_active = True
    idle_cnt = 0
    while follow_active:
        sm.update(50)
        if not sm.updated["logMessage"]:
            print("No msgs")
            continue

        log_msg = json.loads(sm["logMessage"])
        code, data = log_msg["code"], log_msg["data"]
        if code != 0:
            continue

        cmd_msg = None
        people_msgs = [msg for msg in data if msg["label"] == "person" and msg["score"] > YOLO_MIN_CONF]
        if len(people_msgs) != 0:
            person_msg = max(people_msgs, key=lambda a: a["location"]["height"])
            # TODO take person closest to center
            print("found a person")
            pt1 = person_msg["location"]["x"], person_msg["location"]["y"]
            pt2 = person_msg["location"]["x"] + person_msg["location"]["width"], person_msg["location"]["y"] + person_msg["location"]["height"]
            pt1 = (pt1[0] / IMAGE_WIDTH, pt1[1] / IMAGE_HEIGHT)
            pt2 = (pt2[0] / IMAGE_WIDTH, pt2[1] / IMAGE_HEIGHT)
            print("pt1:", pt1, "pt2:", pt2)

            center_x = (pt1[0] + pt2[0]) / 2
            rect_width = abs(pt1[0] - pt2[0])
            rect_height = abs(pt1[1] - pt2[1])
            print("width:", rect_width, "height:", rect_height)
            is_idling = False
            if 0.5 - POS_EPS < center_x < 0.5 + POS_EPS:
                if rect_height < PERSON_HEIGHT_TARGET:
                    cmd_msg = get_control_command(-1, 0)
                else:
                    is_idling = True
                    cmd_msg = get_control_command(0, 0)
            elif 0.5 - POS_EPS > center_x:
                cmd_msg = get_control_command(0, 1)
            else:
                cmd_msg = get_control_command(0, -1)
            if is_idling:
                idle_cnt += 1
            else:
                idle_cnt = 0
            print("Messages:", cmd_msg is None)

        print("cmd_msg:", cmd_msg is not None)
        if cmd_msg is not None:
            print("axes:", cmd_msg.testJoystick.axes)
            pm.send('testJoystick', cmd_msg)

        if idle_cnt > IDLE_CNT_TARGET:
            follow_active = False

        time.sleep(0.050)

if __name__=="__main__":
    find_and_follow_human()
