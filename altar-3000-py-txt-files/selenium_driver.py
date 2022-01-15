from selenium import webdriver
from selenium.common.exceptions import TimeoutException
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.common.by import By
from selenium.webdriver.firefox.webdriver import WebDriver
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions

import time
import pyautogui

import urllib


class SeleniumDriver:
    def __init__(self):
        try:
                
            chromeOptions = webdriver.ChromeOptions() 
            chromeOptions.add_argument("--no-sandbox") 
            chromeOptions.add_argument("--disable-setuid-sandbox") 
            chromeOptions.add_argument("--disable-dev-shm-using") 
            chromeOptions.add_argument("--disable-dev-shm-using") 
            chromeOptions.add_argument("--disable-extensions") 
            chromeOptions.add_argument("--disable-gpu") 
            chromeOptions.add_argument("disable-infobars")
            #chromeOptions.add_argument("headless")

            self.driver = webdriver.Chrome(executable_path='/usr/lib/chromium-browser/chromedriver', chrome_options=chromeOptions)
            
        except Exception as e:
            print(e)
            print("Failed to initialize driver.")
    
    def load_page(self):
        retry = 0
        while retry < 3:
            try:
                # Open the website, add options=opts and uncomment for invisible browser
                self.driver.get("https://8kun.top/random/index.html")
                self.driver.set_page_load_timeout(
                    30
                )  
                
                # set page timeout, throws error if fails to load page
                self.wait = WebDriverWait(
                    self.driver, 15
                )  
                
                # add 10 sec wait between each retrieval
                print("Initialized driver")
                return True
                break
            except:
                retry += 1
                print(str(retry) + " Failed to load, trying again.")
                self.driver.quit()
                return False
        if retry == 3:
            print("Failed to load 8kun page")
            return False

    def write_to_subject(self, string):
        try:
            subject = self.wait.until(
                expected_conditions.presence_of_element_located(
                    (By.XPATH, "//input[@name='subject']")
                )
            )
            subject.send_keys(string)
            print("Wrote to subject")
            return True
        except:
            print("Cannot find subject")
            return False

    def write_to_body(self, string):
        # text area
        try:
            body = self.wait.until(
                expected_conditions.presence_of_element_located((By.ID, "body"))
            )
            # body = self.driver.find_element_by_id("body")
            body.send_keys(string)
            print("Wrote to body")
            return True
        except:
            print("Cannot find body")
            return False

    def add_image(self):
        try:
            add_image = self.wait.until(
                expected_conditions.presence_of_element_located(
                    (By.XPATH, "//div[@class='dropzone']")
                )
            )
            add_image.click()
            pyautogui.write("/home/jetson/of_v20210915_linuxarmv7l_release/apps/myApps/shrine/bin/data/python/images/image.jpg")  # go to folder
            time.sleep(1)
            pyautogui.press("return")
            time.sleep(1)
            print("Added image")
            return True
        except:
            print("Cannot add images")
            return False

    def submit_form(self):
        try:
            # select submit button
            submit_button = self.driver.find_element_by_xpath("//input[@type='submit']")
            submit_button.click()
            print("Submitted form")
            return True
        except:
            print("Form not submitted")
            return False

    def get_captcha_image(self):
        try:
            # get captcha objects
            time.sleep(10)
            # captcha = self.driver.find_element_by_id("captcha_objects")
            captcha = self.wait.until(
                expected_conditions.presence_of_element_located(
                    (By.ID, "captcha_objects")
                )
            )
            captcha_image = captcha.find_element_by_css_selector("img").get_attribute(
                "src"
            )

            # download image
            urllib.request.urlretrieve(captcha_image, "./captcha/captcha.png")
            image_json = {"image": captcha_image}
            print("Retrieved captcha image")
            return image_json
        except Exception as e:
            print(e)
            return False

    def fill_captcha(self, string):
        try:
            captcha = self.wait.until(
                expected_conditions.presence_of_element_located(
                    (By.ID, "captcha_objects")
                )
            )
            captcha_input = captcha.find_element_by_name("captcha_text")
            captcha_input.send_keys(string)

            # submit
            checkbox = self.driver.find_element_by_name("tos_agree")
            checkbox.click()
            print("Filled captcha")
            return True
        except:
            print("Did not fill captcha")
            return False

    def submit_captcha(self):
        # captcha_popup = self.driver.find_element_by_id("captcha_main_box")
        try:
            captcha_popup = self.wait.until(
                expected_conditions.presence_of_element_located(
                    (By.ID, "captcha_main_box")
                )
            )
            submit_post = captcha_popup.find_element_by_id("captcha_pop_submit")
            submit_post.click()
            print("Submit form after captcha")
            return True
        except:
            print("Form not submitted")
            return False

    def run(self):
        print(self.write_to_subject("nice dog"))
        print(self.write_to_body("so cute:)"))
        print(self.add_image())
        has_captcha = self.get_captcha_image()
        if has_captcha:
            self.fill_captcha("some string")
            self.submit_captcha()
        else:
            choice = input(
                "Do you want to submit the post? Press Y and enter if yes: \n"
            )
            if choice == "Y":
                print(self.submit_form())
            else:
                print("Quitting")
                self.driver.quit()  # quit driver


#if __name__ == "__main__":
    #driver = SeleniumDriver()
