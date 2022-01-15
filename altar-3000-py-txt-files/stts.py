#!/usr/bin/python3

import boto3
import speech_recognition as sr
from playsound import playsound
import re

# Amazon Polly CLient
# You need to create your own account via Amazon AWS and insert credentials
polly_client = boto3.Session(
                aws_access_key_id='AKIAYGKWIP4BXSXGJHHG',                     
                aws_secret_access_key='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',
                region_name='eu-west-2').client('polly')

# Speech recognition
r = sr.Recognizer() 

# ===========================================================================
# SYNTHESIZE SPEECH VIA AMAZON POLLY
# ===========================================================================

def synthesize(text):
    try:
        response = polly_client.synthesize_speech(VoiceId= 'Brian',
                                                    OutputFormat= 'mp3', 
                                                    Text = f'{text}',
                                                    Engine = 'neural')

        with open("tts.mp3", 'wb') as s:
            s.write(response['AudioStream'].read())
    except Exception as e:
        print(e)
        pass
    return

# ===========================================================================
# RECORD USER SAYING CAPTCHA AND CONVERT TO TEXT
# ===========================================================================

def hear(duration):
    try:
        with sr.Microphone() as source:
            print("BEEP")
            playsound("beep.wav")
            print("Recording audio")
            # Read audio data from microphone
            audio_data = r.record(source, duration=duration)
            print("Recording finished")
            if audio_data is not None:
                u_human_said = r.recognize_google(audio_data).lower()
                print("Human said (unfiltered): " + u_human_said)
                human_said = clean(u_human_said)
                print("Human said: " + human_said)
            else:
                u_human_said = ""
                human_said = ""
            return human_said, u_human_said
    except Exception as e:
        print(e)
        pass

def clean(string):

    consonants_c = ["sea", "see"]
    consonants_h = "age"
    consonants_i = "eye"
    consonants_j = "jay"
    consonants_o = "oh" 
    consonants_p = ["pea", "pee"]
    consonants_r = "are" 
    consonants_t = "tea"
    consonants_u = "you" 
    consonants_x = "ex" 
    consonants_z = 'zed'
    consonants_y = "why"
    consonants_2 = "to"
    consonants_3 = "free"
    consonants_4 = "for"
    consonants_6 = "sex"
    consonants_8 = "ate"

    for consonant in consonants_c:
        if consonant in string:
            string = string.replace(consonant, "c")
    
    if consonants_h in string:
        string = string.replace(consonants_h, "h")
    if consonants_i in string:
        string = string.replace(consonants_i, "i")
    if consonants_j in string:
        string = string.replace(consonants_j, "j")
    if consonants_o in string:
        string = string.replace(consonants_o, "o")

    for consonant in consonants_p:
        if consonant in string:
            string = string.replace(consonant, "p")

    if consonants_r in string:
        string = string.replace(consonants_r, "r")
    if consonants_t in string:
        string = string.replace(consonants_t, "t")
    if consonants_u in string:
        string = string.replace(consonants_u, "u")
    if consonants_x in string:
        string = string.replace(consonants_x, "x")
    if consonants_y in string:
        string = string.replace(consonants_y, "y")
    if consonants_z in string:
        string = string.replace(consonants_z, "z")
    
    if consonants_2 in string:
        string = string.replace(consonants_2, "2")
    if consonants_3 in string:
        string = string.replace(consonants_3, "3")
    if consonants_4 in string:
        string = string.replace(consonants_4, "4")
    if consonants_6 in string:
        string = string.replace(consonants_6, "6")
    if consonants_8 in string:
        string = string.replace(consonants_8, "8")

    spam = ["the letter", "the number", "the digit", "number", "digit", "letter", "the"]
    numbers = ["zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine"]
    digits = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"]
    
    for word in spam:
        if word in string:
            string = string.replace(word, "")
    for count, number in enumerate(numbers):
        if number in string:
            string = string.replace(number, digits[count])
    
    string = re.sub(r'\W+', '', string)
    string = string.replace(" ", "").lower()
    return string

def record_human():
    captcha = ""
    correct = False
    retries = 0
    playsound("./voice_interactions/submit.mp3")
    while not correct and retries < 3:
        yesno = ""
        captcha, u_captcha = hear(20)
        if captcha is not None:
            synthesize("The captcha is " + u_captcha + ". " + "Is that correct. Yes or No?")
            playsound('tts.mp3')
            yesno = hear(4)
            if yesno is not None:
                if "yes" in yesno:
                    print("Captcha is correct")
                    correct = True
                    playsound("./voice_interactions/success.mp3")
                else:
                    if retries < 2:
                        playsound("./voice_interactions/again.mp3")
                    else: 
                        playsound("./voice_interactions/failure.mp3")
                    retries += 1
                    pass
            else:
                pass
        else:
            pass
    return captcha

# ===========================================================================
