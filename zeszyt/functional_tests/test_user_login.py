from .base import FunctionalTest
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from unittest import skip

class UserLoginTest(FunctionalTest):
    def find_input_boxes(self):
        self.inputbox_name = self.browser.find_element_by_id('id_username')
        self.inputbox_password = self.browser.find_element_by_name('password')
        self.login_button = self.browser.find_element_by_name('login')

    def test_login_page_exist(self):
        # Użytkownik wchodzi na stronę główną.
        # Widzi panel logowania
        self.wait_for(lambda: self.assertIn("Strona logowania",
            self.browser.find_element_by_tag_name('body').text
        ))

    def test_wrong_login(self):
        # Loguje się błędnym loginem
        self.find_input_boxes()
        self.inputbox_name.send_keys('wrong login')
        self.inputbox_password.send_keys('pass1')
        self.login_button.click()
        # Widzi powiadomienie, że login jest błędny
        self.wait_for(lambda: self.assertIn("Błędny login lub hasło",
            self.browser.find_element_by_tag_name('body').text
        ))
    def test_wrong_pass(self):
        # Loguje się błędnym hasłem
        self.find_input_boxes()
        self.inputbox_name.send_keys('user1')
        self.inputbox_password.send_keys('pass2')
        self.login_button.click()
        self.wait_for(lambda: self.assertIn("Błędny login lub hasło",
            self.browser.find_element_by_tag_name('body').text
        ))
    @skip
    def test_not_active_user_login(self):

        # Loguje się błędnym hasłem
        self.find_input_boxes()
        self.inputbox_name.send_keys('user1')
        self.inputbox_password.send_keys('pass2')
        self.login_button.click()
        self.wait_for(lambda: self.assertIn("Błędny login lub hasło",
            self.browser.find_element_by_tag_name('body').text
        ))

    def correct_login(self):
        # Loguje się błędnym hasłem
        self.find_input_boxes()
        self.inputbox_name.send_keys('user1')
        self.inputbox_password.send_keys('pass1')
        self.login_button.click()
        self.wait_for(lambda: self.assertIn("Panel",
            self.browser.find_element_by_tag_name('title').text
        ))