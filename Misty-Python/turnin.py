from mistyPy.Robot import Robot
from mistyPy.Events import Events
import time
from functools import partial
import random

import sounddevice
import soundfile
from scipy.io.wavfile import write

import whisper
print('Loading transcription model.')
model = whisper.load_model("small.en")

# def say_text(text: str):
#     print(text)
#     engine.say(text)
#     engine.runAndWait()

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
    wait()

    misty.DisplayImage("e_DefaultContent.jpg")
    misty.MoveArm("Both", 0, 100)
    misty.MoveHead(0, 0, 0, 100)
    wait(2000)

def handle_bumper_event(misty: Robot, data):
    if data["message"]["sensorId"] == 'bfr':
        misty.MoveArms(90, 90, 100)
    elif data["message"]["sensorId"] == 'brr':
        misty.MoveArms(-90, -90, 100)

def handle_head_sensor_event(misty: Robot, data):
    if data["message"]["sensorPosition"] == 'HeadFront' :
        misty.ChangeLED(0, 255, 0) 
        misty.Drive(50, 0)
        wait(2000) 
        misty.Stop()
    elif data["message"]["sensorPosition"] == "HeadLeft":
        misty.Drive(30, 45)  
        misty.ChangeLED(255, 255, 0)
        wait(2000)
        misty.Stop()
    elif data["message"]["sensorPosition"] =="HeadRight":
        misty.Drive(30, -45) 
        misty.ChangeLED(0, 255, 255)
        wait(2000)
        misty.Stop() 

def detect_human(misty: Robot, data):
    if data["message"]["description"] == "person":
        misty.MoveArms(-90, -90, 100)
        wait()
        misty.MoveArms(-90, 90, 100)
        wait()
        misty.MoveArms(-90, -90, 100)
        wait()
        misty.MoveArms(-90, 90, 100)
        wait()
        misty.Speak("Hello, What's Up baby!")
        misty.stop_object_detector()

def start_task_2(misty: Robot):
    misty.StartObjectDetector()
    misty.RegisterEvent(event_name='bump_event', event_type=Events.BumpSensor,  callback_function = partial(handle_bumper_event,misty), keep_alive=True)
    misty.RegisterEvent(event_name='touch', event_type=Events.TouchSensor, callback_function = partial(handle_head_sensor_event,misty), keep_alive=True)
    misty.RegisterEvent(event_name='object_detection_event', event_type =Events.ObjectDetection, callback_function= partial(detect_human,misty), keep_alive=False)
    
    misty.KeepAlive()
    while True:
        wait()

import pyttsx3
def say_text(text: str, misty: Robot=None):
    if misty==None:
        engine = pyttsx3.init()
        engine.say(text)
        engine.runAndWait()
    else:
        misty.Speak(text)
        wait(2500/25*len(text))

def get_audio(file_name='output.wav', f_s=50000, seconds=2, misty:Robot=None):
    if misty != None:
        misty.ChangeLED(0, 0, 255)

    my_recording = sounddevice.rec(int(seconds*f_s), f_s, 2)
    sounddevice.wait()
    write(file_name, f_s, my_recording)

    if misty != None:
        misty.ChangeLED(0, 0, 0)

    return my_recording

# def play_audio(file_name='output.wav', f_s=50000):
#     audio, f_s = soundfile.read(file_name, dtype='float32')
#     sounddevice.play(audio, f_s)
#     sounddevice.wait()

def get_translation(file_name='output.wav'):
    result = model.transcribe(file_name, fp16=False)
    
    return result["text"].strip()

def conversation(starting_question: str, repeat_question: str, key_phrases: dict, misty: Robot=None):
    say_text(starting_question, misty)

    key_phrase_identified = False

    while not key_phrase_identified:
        get_audio(misty=misty)
        spoken_words = get_translation()

        for key in key_phrases:
            key_phrases[key] = key in spoken_words.lower()

            if key_phrases[key]:
                key_phrase_identified = True
        
        if not key_phrase_identified:
            wait(100)
            say_text('I am deeply sorry. I heard: ' + spoken_words, misty)
            say_text(repeat_question, misty)
    
    return key_phrases

def translate_results(results: dict):
    first_true = ''

    for key in results:
        if results[key]:
            first_true = key
            break
    
    return first_true

def start_task_3_a(misty: Robot=None):
    like_pie = translate_results(conversation("Do you like pie?", "I didn't hear a yes, love, no, or never. Please repeat.", {'yes': False, 'love': False, 'no': False, 'never': False}, misty))
    want_pie_to_face = translate_results(conversation("Would you like a pie to the face?", "I didn't hear a yes, yeah, no, or how dare you. Please repeat.", {'yes': False, 'yeah': False, 'no': False, 'dare': False}, misty))

    text_to_say = ''
    if (like_pie == 'yes' or like_pie == 'love') and (want_pie_to_face == 'yes' or want_pie_to_face == 'yeah'):
        text_to_say = "You're crazy!"
    elif (like_pie == 'yes' or like_pie == 'love') and want_pie_to_face == 'no':
        text_to_say ="Well here's one anyway."
    elif (like_pie == 'yes' or like_pie == 'love') and want_pie_to_face == 'dare':
        text_to_say = "How can I not dare? You've left yourself wide open."
    elif (like_pie == 'no' or like_pie == 'never'):
        text_to_say = "That's impossible, how can you not like pie?!"
    else:
        text_to_say = "Well this is awkward. My programmer, John, was stupid enough to not cover this combination. Please reprimand him for me. smh."

    say_text(text_to_say, misty)

def joke_context(initial_statement: str, punchline: str, misty: Robot=None):
    say_text(initial_statement, misty)
    wait(200)
    say_text(punchline, misty)

    get_audio(misty=misty)
    spoken_words = get_translation()

    print(spoken_words + '\n')

    return 'haha' in spoken_words or 'ha ha' in spoken_words or 'heh' in spoken_words

def start_task_3_b(misty: Robot=None):
    jokes = [
        ['Why did the banana go to the doctor?', "It wasn't peeling well."],
        ['Did you hear about the 2 people who stole a calendar?', "They each got 6 months."],
        ["Why didn't the lifeguard rescue the hippie?", "Cause he was too far out man."],
        ["Hey Miller, help I need some jokes!", "You're mother, gaht ehm."],
        ["Hey Miller, are you paying attention?", "No, I'm Miller."],
        ["Why are ghosts such bad liars?", "Because they are so easy to see through."],
        ["Why didn't the orange win the race?", "Because it ran out of juice."],
        ["What do you call a music teacher with problems?", "A very trebled man."]
    ]

    jokes = random.sample(jokes, k=4)

    funny = []
    for joke in jokes:
        funny.append(int(joke_context(joke[0], joke[1], misty)))

    percent_funny = int(sum(funny)/len(funny)*100)
    if percent_funny == 0:
        say_text("Apparently you didn't find any of my jokes funny. This greatly saddens me.", misty)
    elif percent_funny < 100:
        funny_indexes = []
        for index in range(len(funny)):
            if funny[index]:
                funny_indexes.append(index)
        
        joke_string = ''
        for index, funny_index in enumerate(funny_indexes):
            joke_string += ' '.join(jokes[funny_index])

            if index < len(funny_indexes) - 2:
                joke_string += ' joke. The '
            elif index < len(funny_indexes) - 1:
                joke_string += ' joke and the '

        say_text(f"You thought the {joke_string} joke was funny. I'll try harder next time.", misty)
    elif percent_funny == 100:
        say_text(f"You really thought all of my jokes were funny? You're too kind.", misty)

def poll_ips():
    start = "192.168.0."

    valid_ips = []
    for index in range(100, 255):
        try:
            Robot(start + f'{index}')
            valid_ips.append(index)
        except:
            failed = True
    
    print(valid_ips)

if __name__ == "__main__":
    ipAddress = "192.168.0.113"
    
    try:
        misty = Robot(ipAddress)
        # misty.StopSpeaking()
        start_task_1(misty)
        # start_task_2(misty)
        # start_task_3_a(misty)
        # start_task_3_b(misty)
    except:
        print(f"Connection to {ipAddress} failed.")


