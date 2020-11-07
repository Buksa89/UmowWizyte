from django.contrib.auth.models import User
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from unittest import skip
import time
from .base import FunctionalTest

class UserLoginTest(FunctionalTest):
    def find_input_boxes(self):
        self.inputbox_name = self.browser.find_element_by_id('id_username')
        self.inputbox_password = self.browser.find_element_by_name('password')
        self.login_button = self.browser.find_element_by_name('login')

    def test_wrong_login(self):
        # Loguje się błędnym loginem
        self.find_input_boxes()
        self.inputbox_name.send_keys('wrong login')
        self.inputbox_password.send_keys('pass')
        time.sleep(2)
        self.login_button.click()
        # Widzi powiadomienie, że login jest błędny
        self.wait_for(lambda: self.assertIn("Błędny login lub hasło",
            self.browser.find_element_by_tag_name('body').text
        ))
    def test_wrong_pass(self):
        # Loguje się błędnym hasłem
        self.find_input_boxes()
        self.inputbox_name.send_keys('user')
        self.inputbox_password.send_keys('wrong_pass')
        time.sleep(2)
        self.login_button.click()
        self.wait_for(lambda: self.assertIn("Błędny login lub hasło",
            self.browser.find_element_by_tag_name('body').text
        ))

    def test_not_active_user_login(self):
        # Loguje się jako zablokowany użytkownik
        self.find_input_boxes()
        self.inputbox_name.send_keys('user_not_active')
        self.inputbox_password.send_keys('pass_not_active')
        time.sleep(2)
        self.login_button.click()
        self.wait_for(lambda: self.assertIn("Konto zablokowane",
            self.browser.find_element_by_tag_name('body').text
        ))

    def test_correct_login(self):
        # Loguje się
        self.find_input_boxes()
        self.inputbox_name.send_keys('user')
        self.inputbox_password.send_keys('pass')
        time.sleep(2)
        self.login_button.click()
        self.wait_for(lambda: self.assertIn("Panel",
            self.browser.find_element_by_tag_name('body').text
        ))