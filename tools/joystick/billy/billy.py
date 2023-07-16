from talk_io import talk, play_sound, record_audio, audio_transcribe, chat_completion, initialize_audio, terminate_audio, MIC_DEVICE_INDEX, ACTIONS
from moving import find_and_follow_human
from enum import Enum
import os
import time
import multiprocessing
import cereal.messaging as messaging
from cereal.visionipc import VisionIpcClient, VisionStreamType
from system.camerad.snapshot.snapshot import extract_image, jpeg_write

pm = messaging.PubMaster(['bodyStatus'])
vipc_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_DRIVER, True)

class BillyState(Enum):
  IDLE = 0
  SEEKING = 1
  WALKING = 2
  STOPPED = 3

  INTRO = 5
  LISTENING = 6
  THINKING = 7

  PHOTO = 8
  JOKE = 9
  SONG = 10
  NOTHING = 11

def stopped():
  output_path = talk(0)
  play_sound(output_path)
  return BillyState.LISTENING

def listening():
  output_file_name = record_audio(index=MIC_DEVICE_INDEX, verbose=True)
  global transcript
  transcript = audio_transcribe(verbose=True, output_file_name=output_file_name)
  return BillyState.THINKING

def seeking():
  find_and_follow_human()
  return BillyState.STOPPED

def thinking():
  action = chat_completion(transcript)
  print(ACTIONS[action])
  if action == 1:
    return BillyState.PHOTO
  elif action == 2:
    return BillyState.JOKE
  elif action == 3:
    return BillyState.SONG
  elif action == 4:
    return BillyState.NOTHING
  else:
    return BillyState.STOPPED

def send_body_status(res):
  msg = messaging.new_message()
  msg.bodyStatus = res
  pm.send('bodyStatus', msg)

def take_snapshot(filename):
  img = vipc_client.recv()
  img = extract_image(img.flatten(), vipc_client.width, vipc_client.height, vipc_client.stride, vipc_client.uv_offset)
  jpeg_write(filename, img)

def print_snapshot(filename):
  printer_ip = os.environ["PRINTER_ADDR"]
  os.system(f"ipptool -tv -f {filename} {printer_ip} /data/openpilot/tools/joystick/billy/printjob.ipp")

def play_song():
  play_sound('/data/openpilot/tools/joystick/billy/zelda.mp3')


def photo():
  output_path = talk(1)
  play_sound(output_path)
  send_body_status(1)
  time.sleep(6)
  send_body_status(0)

  filename = '/data/openpilot/snapshot.jpg'
  take_snapshot(filename)
  print_snapshot(filename)

  return BillyState.IDLE

def joke():
  output_path = talk(2)
  play_sound(output_path)
  return BillyState.STOPPED

def song():
  output_path = talk(3)
  play_sound(output_path)

  p = multiprocessing.Process(target=play_song, args=())
  p.start()
  time.sleep(8)
  p.join()

  return BillyState.STOPPED

def nothing():
  output_path = talk(4)
  play_sound(output_path)
  return BillyState.STOPPED

def idle():
  return BillyState.IDLE

billy_functions = {
  BillyState.SEEKING: seeking,
  BillyState.STOPPED: stopped,
  BillyState.LISTENING: listening,
  BillyState.THINKING: thinking,
  BillyState.PHOTO: photo,
  BillyState.JOKE: joke,
  BillyState.SONG: song,
  BillyState.NOTHING: nothing,
  BillyState.IDLE: idle,
}


if __name__ == "__main__":
  send_body_status(0)
  initialize_audio()
  billy_state = BillyState.STOPPED

  if not vipc_client.is_connected():
    vipc_client.connect(True)

  while True:
    print(billy_state)
    billy_state = billy_functions[billy_state]()
    time.sleep(0.1)
    if billy_state == BillyState.IDLE:
      break

  terminate_audio()
