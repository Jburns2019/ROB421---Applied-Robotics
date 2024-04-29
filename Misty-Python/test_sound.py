import sounddevice
import soundfile
from scipy.io.wavfile import write
import pyttsx3
import time
import random

import whisper
print('Loading transcription model.')
model = whisper.load_model("small.en")
print('Loading text to speach engine.')
engine = pyttsx3.init()

def say_text(text: str):
    print(text)
    engine.say(text)
    engine.runAndWait()

def get_audio(file_name='output.wav', f_s=50000, seconds=3):
    my_recording = sounddevice.rec(int(seconds*f_s), f_s, 2)
    sounddevice.wait()
    write(file_name, f_s, my_recording)

    return my_recording

# def play_audio(file_name='output.wav', f_s=50000):
#     audio, f_s = soundfile.read(file_name, dtype='float32')
#     sounddevice.play(audio, f_s)
#     sounddevice.wait()

def get_translation(file_name='output.wav'):
    result = model.transcribe(file_name, fp16=False)
    
    return result["text"].strip().lower()

def conversation(starting_question: str, repeat_question: str, key_phrases: dict):
    say_text(starting_question)
    key_phrase_identified = False

    while not key_phrase_identified:
        get_audio()
        spoken_words = get_translation()

        for key in key_phrases:
            key_phrases[key] = key in spoken_words

            if key_phrases[key]:
                key_phrase_identified = True
        
        if not key_phrase_identified:
            say_text('I am deeply sorry. I heard: ' + spoken_words)
            say_text(repeat_question)
    
    return key_phrases

def translate_results(results: dict):
    first_true = ''

    for key in results:
        if results[key]:
            first_true = key
            break
    
    return first_true

def start_task_3_a():
    like_pie = translate_results(conversation("Do you like pie?", "I didn't hear a yes, love, no, or never. Please repeat.", {'yes': False, 'love': False, 'no': False, 'never': False}))
    want_pie_to_face = translate_results(conversation("Would you like a pie to the face?", "I didn't hear a yes, yeah, no, or how dare you. Please repeat.", {'yes': False, 'yeah': False, 'no': False, 'dare': False}))

    if (like_pie == 'yes' or like_pie == 'love') and (want_pie_to_face == 'yes' or want_pie_to_face == 'yeah'):
        say_text("You're crazy!")
    elif (like_pie == 'yes' or like_pie == 'love') and want_pie_to_face == 'no':
        say_text("Well here's one anyway.")
    elif (like_pie == 'yes' or like_pie == 'love') and want_pie_to_face == 'dare':
        say_text("How can I not dare? You've left yourself wide open.")
    elif (like_pie == 'no' or like_pie == 'never'):
        say_text("That's impossible, how can you not like pie?!")
    else:
        say_text("Well this is awkward. My programmer, John, was stupid enough to not cover this combination. Please reprimand him for me. smh.")

def joke_context(initial_statement: str, punchline: str):
    say_text(initial_statement)
    time.sleep(.2)
    say_text(punchline)

    get_audio()
    spoken_words = get_translation()

    print(spoken_words + '\n')

    return 'haha' in spoken_words or 'ha ha' in spoken_words or 'heh' in spoken_words

def start_task_3_b():
    jokes = [
        ['Why did the banana go to the doctor?', "It wasn't peeling well."],
        ['Did you hear about the 2 people who stole a calendar?', "They each got 6 months."],
        ["Why didn't the lifeguard rescue the hippie?", "Cause he was too far out man."],
        ["Hey Miller, help I need some jokes!", "You're mother, got em."],
        ["Hey Miller, are you paying attention?", "No, I'm Miller."],
        ["Why are ghosts such bad liars?", "Because they are so easy to see through."],
        ["Why didn't the orange win the race?", "Because it ran out of juice."],
        ["What do you call a music teacher with problems?", "A very trebled man."]
    ]

    jokes = random.sample(jokes, k=4)

    funny = []
    for joke in jokes:
        funny.append(int(joke_context(joke[0], joke[1])))

    percent_funny = int(sum(funny)/len(funny)*100)
    if percent_funny == 0:
        say_text("Apparently you didn't find any of my jokes funny. This greatly saddens me.")
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

        say_text(f"You thought the {joke_string} joke was funny. I'll try harder next time.")
    elif percent_funny == 100:
        say_text(f"You really thought all of my jokes were funny? You're too kind.")


if __name__ == "__main__":
    # start_task_3_a()
    start_task_3_b()