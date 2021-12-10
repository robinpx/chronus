#!/usr/bin/python3

import time
import os.path
from os import path
from time import sleep
from playsound import playsound
from pythonosc import udp_client

print("Loading...")

import predictit
import prophet
import stts
import selenium_driver
import image_search

print("Loaded.")

# Scheduler
starttime = time.time()

# Adding first person prophesizer seed to text generator
mood = "I, the prophet proclaim that"

# Web scrapper
driver = selenium_driver.SeleniumDriver()

# OSC client
client = udp_client.SimpleUDPClient("127.0.0.1", 3001)

# ===========================================================================
# Step 1: Retrieve today's bet headlines from PredictIt
# ===========================================================================

predictit.all_market_names()

# ===========================================================================


while True:
    try:

        # ===========================================================================
        # Step 2: Pick random headline
        # ===========================================================================
        headline = predictit.pick_random_bet()
        # Send headline to openFrameworks 
        client.send_message("/headline", headline.upper())
        
        print("====================================================================")
        print("HEADLINE: " + headline)
        print("====================================================================")

        stts.synthesize(headline)
        playsound('tts.mp3')

        # ===========================================================================
        # Step 3: Generate a prophecy
        # ===========================================================================
        prohecy = ""
        prophecy = prophet.generate_prophecy(headline + " " + mood)
        # Send prophecy to openframeworks
        client.send_message("/prophecy", prophecy.upper())
        print("====================================================================")
        print("PROPHECY: " + "\n")
        print(prophecy)
        print("====================================================================")

        # ===========================================================================
        # Step 3: Speak prophecy aloud
        # ===========================================================================

        stts.synthesize(prophecy)
        playsound('tts.mp3')
        sleep(1)


        # ===========================================================================
        # Step 4: Record captcha input from human
        # ===========================================================================

        # Ask if human wants to share the prophecy
        print("Would you like to share?")
        print("====================================================================")
        playsound("./voice_interactions/share.mp3")
        sleep(.5)
        confirm = ""

        confirm = stts.hear(4)

        if confirm is not None:
            if "yes" in confirm or "yeah" in confirm or "ye" in confirm or "yeh" in confirm:
                print("Sharing...")
                
                playsound("./voice_interactions/sharing.mp3")

                # ===========================================================================
                # Step 5: Submit post to 8kun 
                # ===========================================================================
                success_driver = driver.load_page()
                if success_driver:
                    driver.write_to_subject(headline)
                    driver.write_to_body(prophecy)

                    # Find a nice image to accompany the prophecy
                    image_search.find_image(headline)
                    sleep(1)
                    if path.exists("./images/image.jpg"):
                        client.send_message("/image", True)
                        driver.add_image()

                    # Submit form
                    success_submit = driver.submit_form()
                    sleep(1)
                    if success_submit:
                        has_captcha = driver.get_captcha_image() 
                        if has_captcha:
                            client.send_message("/captcha", True)
                            human_captcha = stts.record_human()
                            driver.fill_captcha(human_captcha)
                            success_captcha = driver.submit_captcha()
                            if success_captcha:
                                client.send_message("/success", True)
                                playsound("./voice_interactions/success.mp3")
                            else:
                                client.send_message("/success", False)
                                playsound("./voice_interactions/failure.mp3")
                        else:
                            client.send_message("/success", True)
                            playsound("./voice_interactions/success.mp3")
                    else:
                        client.send_message("/success", False)
                        playsound("./voice_interactions/failure.mp3")
                else:
                    client.send_message("/success", False)
                    playsound("./voice_interactions/failure.mp3")

        # Tell shrine screen to reload
        print("Sending Reset Prompt")
        client.send_message("/reset", True)    
    
    except Exception as e:
        print(e)
        playsound("./voice_interactions/failure.mp3")
        client.send_message("/success", False)
        sleep(2)
        client.send_message("/reset", True) 
        print("====================================================================")
        print("Someting went wrong. Will return shortly.")
        print("====================================================================")

    # Execute every 5 mins
    print("Returning in 3 minutes")
    time.sleep(180.0 - ((time.time() - starttime) % 60.0))
    
    # ===========================================================================