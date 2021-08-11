from selenium import webdriver
from selenium.common.exceptions import TimeoutException
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.common.by import By
from selenium.webdriver.firefox.webdriver import WebDriver
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions

import time


class SeleniumDriver:
    def __init__(self):
        retry = 0
        while retry < 3:
            try:
                # opts = webdriver.FirefoxOptions()  # driver is invisible
                # opts.headless = True
                self.driver = webdriver.Firefox()  # Open the website
                self.driver.get("https://8kun.top/random/index.html")
                # self.driver.navigate().refresh()
                self.driver.set_page_load_timeout(
                    30
                )  # set page timeout, throws error if fails to load page
                self.wait = WebDriverWait(
                    self.driver, 10
                )  # add 10 sec wait between each retrieval
                print("Initialized driver")
                break
            except:
                retry += 1
                print(str(retry) + " Failed to load, trying again.")
                self.driver.quit()
        if retry == 3:
            print("Failed to initialize")

    def _parse(self, string):
        # parse string to be understood by selenium
        return string

    def write_to_body(self, string):
        # text area
        try:
            body = self.wait.until(
                expected_conditions.presence_of_element_located((By.ID, "body"))
            )
            # body = self.driver.find_element_by_id("body")
            parse_string = self._parse(string)
            body.send_keys(parse_string)
            print("Writing to body")
            # select submit button
            submit_button = self.driver.find_element_by_xpath("//input[@type='submit']")
            submit_button.click()
            print("Submitted body")
            return True
        except:
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
            print(captcha_image)
            image_json = {"image": captcha_image}
            print("Retrieved captcha image")
            return image_json
        except:
            return False

    def submit_captcha(self):
        try:
            captcha = self.wait.until(
                expected_conditions.presence_of_element_located(
                    (By.ID, "captcha_objects")
                )
            )
            captcha_input = captcha.find_element_by_name("captcha_text")
            captcha_input.send_keys("cool")

            # submit
            checkbox = self.driver.find_element_by_name("tos_agree")
            checkbox.click()
            print("Filled captcha")
            return True
        except:
            return False

    def submit_post(self):
        # captcha_popup = self.driver.find_element_by_id("captcha_main_box")
        try:
            captcha_popup = self.wait.until(
                expected_conditions.presence_of_element_located(
                    (By.ID, "captcha_main_box")
                )
            )
            submit_post = captcha_popup.find_element_by_id("captcha_pop_submit")
            # submit_post.click()
            return True
        except:
            return False

    def run(self):
        print(
            self.write_to_body(
                """nice a long !234jiadlj#4592031
        dfajdlk \n jkasdl \rf jas"""
            )
        )
        print(self.get_captcha_image())
        print(self.submit_captcha())
        print(self.submit_post())


if __name__ == "__main__":
    driver = SeleniumDriver()
    driver.run()
