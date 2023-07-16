import numpy as np
import os
import openai
import pyaudio
import wave
from elevenlabs import generate, save, set_api_key
import time

# keys
openai_key = os.getenv('OPENAI_KEY')
elevenlabs_key = os.getenv('ELEVENLABS_KEY')

assert openai_key and elevenlabs_key
openai.api_key = openai_key
set_api_key(elevenlabs_key)

MIC_DEVICE_INDEX = 0

ACTIONS = {
    1: 'take photo',
    2: 'tell a joke',
    3: 'play a song',
    4: 'none of the above'
}

TALKING = {
  0: """Hi! My name is Billy, nice to meet you! I can do some basic things,
  take photos, tell a joke, or play a song. What do you Want me to Do?""",

  1: """Awesome! Let's take a photo! Smile!""",
  2: """Okay! Here is a joke for you, I hope it makes you laugh.""",
  3: """Okay! Here is my favorite song for you.""",
  4: """Okay! I will just sit here and do nothing."""
}

def record_audio(index=0, output_file_name="recordedFile.wav",
                 max_record_seconds=10, no_voice_threshold=100,
                 voice_threshold=300, verbose=True):
  FORMAT = pyaudio.paInt16
  CHANNELS = 1
  RATE = 44100
  CHUNK = 512

  audio = pyaudio.PyAudio()
  stream = audio.open(format=FORMAT, channels=CHANNELS,
                rate=RATE, input=True,
                frames_per_buffer=CHUNK)

  if verbose: print ("recording started")
  Recordframes = []

  there_is_voice = 0
  there_is_no_voice = 0
  i = 0
  max_listen = RATE / CHUNK * max_record_seconds
  while i < max_listen:
    i+=1
    data = stream.read(CHUNK)
    Recordframes.append(data)
    data_numpy = np.frombuffer(data, dtype=np.int16)
    level = np.abs(data_numpy).mean()
    if level < no_voice_threshold:
      there_is_no_voice += 1
    if level > voice_threshold:
      there_is_voice += 1
      there_is_no_voice = 0
    if there_is_no_voice > 200 and there_is_voice > 5:
      break
    if verbose: print(i, level, there_is_voice, there_is_no_voice)

  if verbose: print ("recording stopped")

  stream.stop_stream()
  stream.close()
  audio.terminate()

  waveFile = wave.open(output_file_name, 'wb')
  waveFile.setnchannels(CHANNELS)
  waveFile.setsampwidth(audio.get_sample_size(FORMAT))
  waveFile.setframerate(RATE)
  waveFile.writeframes(b''.join(Recordframes))
  waveFile.close()
  return output_file_name

def audio_transcribe(output_file_name="recordedFile.wav", verbose=False):
  for _ in range(3):
    try:
      with open(output_file_name, "rb") as audio_file:
        transcript = openai.Audio.transcribe("whisper-1", audio_file)
      if verbose: print(transcript)
      return transcript
    except:
      pass
  return "ERROR"

def chat_completion(transcript, model="gpt-3.5-turbo"):
  actions_str = ''.join([f"[{k}]: {v} , \n" for k,v in ACTIONS.items()])
  preprompt = "You are a robot called Billy, made by comma.ai."
  prompt =  """
  Given the following [request] give the index of the most relevant action from this list
  """ + actions_str + \
  """
  Only output the number, do not describe the action or discuss with the user.
  Output format: ACTION: [number]

  This is the [request]
  """ + transcript['text']

  messages=[
    {"role": "system", "content": preprompt},
    {"role": "user", "content": prompt}
  ]
  for _ in range(3):
    try:
      completion = openai.ChatCompletion.create(model="gpt-3.5-turbo", messages=messages)
      completion = completion.choices[0].message.content
      return int(completion.replace("ACTION: ", ""))
    except:
      pass

  return "ERROR"

def talk(key, output_path='elevenlab.wav'):
  output_path = f'elevenlab_{key}.wav'

  if os.path.exists(output_path):
    return output_path

  for _ in range(3):
    try:
      audio = generate(text=TALKING[key], voice="Arnold", model="eleven_monolingual_v1")
      save(audio, output_path)
      return output_path

    except Exception as e:
      print(e)
      pass

  return "ERROR"

def play_sound(output_path):
  print("Playing sound", output_path)
  os.system(f"ffplay -nodisp -autoexit {output_path} 2>/dev/null")
