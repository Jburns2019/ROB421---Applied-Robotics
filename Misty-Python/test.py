from mistyPy.Robot import Robot
from mistyPy.Events import Events
import time

import sounddevice
import soundfile
from scipy.io.wavfile import write

import whisper
model = whisper.load_model("base")

# print(current_response)
# print(current_response.status_code)
# print(current_response.json())
# print(current_response.json()["result"])

def example_string():
    text = """
    <speak>
        I can talk at different speeds.
        <prosody rate=\"fast\">I can talk really fast</prosody>.
        <prosody rate=\"x-slow\">Or I can talk really slow</prosody>.
        I can talk at different volumes.
        This is the default volume.
        <prosody volume=\"x-low\" pitch=\"x-low\">I can whisper.</prosody>.
        <prosody volume=\"x-loud\" pitch=\"x-high\">Or I can yell!</prosody>
    </speak>
    """
    text = text.replace('\n', '').replace('\t', ' ')
    while '  ' in text:
        text = text.replace('  ', ' ')

    # current_response = misty.Speak(text[1:], flush=True, utteranceId= "First")
    return text[1:]

def wait(timems: int=1000):
    start_time = time.time()*1000
    current_time = time.time()*1000
    while current_time-start_time < timems:
        current_time = time.time()*1000

def start_task_1(misty: Robot):
    misty.DisplayImage("e_DefaultContent.jpg")
    misty.MoveArm("Both", 0, 100)
    misty.MoveHead(0, 0, 0, 100)
    wait(2000)

    misty.Speak("I am now crossing the road.")
    misty.ChangeLED(255, 0, 0)
    misty.DisplayImage("e_Rage4.jpg")
    
    misty.MoveHead(0, 0, -90, 100)
    wait(1000)
    misty.MoveArms(90, -90, 100, 100)
    misty.MoveHead(0, 0, 90, 100)
    wait(1000)
    misty.MoveHead(0, 0, 0, 100)
    wait(1000)

    misty.Drive(50, 0)
    wait(5000)
    misty.Stop()
    misty.Drive(0, -100)
    wait(6400)
    misty.Stop()
    misty.Drive(50, 0)
    wait(5000)
    misty.Stop()
    misty.Drive(0, -100)
    wait(6400)
    misty.Stop()
    
    misty.DisplayImage("e_Disgust.jpg")
    misty.Speak("Thank you for not hitting me!")
    misty.ChangeLED(0, 255, 0)

    misty.DisplayImage("e_DefaultContent.jpg")
    misty.MoveArm("Both", 0, 100)
    misty.MoveHead(0, 0, 0, 100)
    wait(2000)

def was_bumped(data):
    print('I was bumped.')

def start_task_2(misty: Robot):
    misty.RegisterEvent(event_name='bump_event', event_type=Events.BumpSensor, callback_function=was_bumped, keep_alive=False)

def start_task_3(misty: Robot=None):
    f_s = 50000
    seconds = 2.5

    audio, f_s = soundfile.read('voice_prompt.wav', dtype='float32')
    sounddevice.play(audio, f_s)
    sounddevice.wait()

    print('Recording audio now.')

    my_recording = sounddevice.rec(int(seconds*f_s), f_s, 2)
    sounddevice.wait()
    write('output.wav', f_s, my_recording)

    print('Playing audo.')

    audio, f_s = soundfile.read('output.wav', dtype='float32')
    sounddevice.play(audio, f_s)
    sounddevice.wait()

    print('Transcribing audio.')

    result = model.transcribe("C:\\Users\\Kimberlie P17\\Documents\\ActualDocuments\\SchoolRecords\\Spring_Term\\Spring_Term_2024\\Misty-Python\\output.wav", verbose = True, fp16=False)
    print(result["text"])

if __name__ == "__main__":
    # ipAddress = "192.168.0.109"
    # try:
        # misty = Robot(ipAddress)
        # start_task_1(misty)
        # start_task_2(misty)
    start_task_3()
    # except:
        # print(f"Connection to {ipAddress} failed.")
