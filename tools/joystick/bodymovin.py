import json

import cereal.messaging as messaging
from cereal import log
from cereal.visionipc import VisionIpcClient, VisionStreamType

IMAGE_HEIGHT, IMAGE_WIDTH = 416, 640
POS_EPS = 0.1
PERSON_WIDTH_TARGET = 0.4


def get_control_command(x, y):
    x = max(-1.0, min(1.0, x))
    y = max(-1.0, min(1.0, y))

    dat = messaging.new_message('testJoystick')
    dat.testJoystick.axes = [x, y]
    dat.testJoystick.buttons = [False]

    return dat

def asleep_face_msg():
    dat = messaging.new_message('bodyFace')
    dat.bodyFace = log.BodyFace.sleep
    return dat 

def awake_face_msg():
    dat = messaging.new_message('bodyFace')
    dat.bodyFace = log.BodyFace.awake
    return dat 

{'label': 'person', 'location': {'height': 44.655364990234375, 'width': 21.4017333984375, 'x': 279.86962890625, 'y': 81.7782211303711}, 'score': 0.7005926966667175}

def run_loop():
    pm = messaging.PubMaster(['testJoystick', 'bodyFace'])
    sm = messaging.SubMaster(['logMessage'])
        
    while True:
        sm.update(50)
        if not sm.updated["logMessage"]:
            print("No msgs")
            continue

        log_msg = json.loads(sm["logMessage"])
        code, data = log_msg["code"], log_msg["data"]
        if code != 0:
            continue

        cmd_msg, face_msg = None, None
        assert type(log_msg) == list
        person_msg = next((msg for msg in data if msg["label"] == "person"), None)
        if person_msg == None:
            face_msg = asleep_face_msg()
        else:
            pt1 = person_msg["location"]["x"], person_msg["location"]["y"]
            pt2 = person_msg["location"]["x"] + person_msg["location"]["width"], person_msg["location"]["y"] + person_msg["location"]["height"]
            pt1 = (pt1[0] / IMAGE_WIDTH, pt1[1] / IMAGE_HEIGHT)
            pt2 = (pt2[0] / IMAGE_WIDTH, pt2[1] / IMAGE_HEIGHT)
            print("pt1:", pt1, "pt2:", pt2)

            center_x = (pt1[0] + pt2[0]) / 2
            rect_width = abs(pt1[0] - pt2[0])
            face_msg = awake_face_msg()
            if 0.5 - POS_EPS < center_x < 0.5 + POS_EPS:
                if pt1[1] > POS_EPS and pt2[1] < 1 - POS_EPS and rect_width > PERSON_WIDTH_TARGET:
                    cmd_msg = get_control_command(-1, 0)
            elif 0.5 - POS_EPS > center_x:
                cmd_msg = get_control_command(0, 1)
            else:
                cmd_msg = get_control_command(0, -1)
            print("Messages:", cmd_msg is None, face_msg is None)

        if cmd_msg is not None:
            pm.send('testJoystick', cmd_msg)
        if face_msg is not None:
            pm.send('bodyFace', face_msg)

if __name__=="__main__":
    run_loop()