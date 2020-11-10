from django.contrib.auth.models import User
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from unittest import skip
import time
from .base import FunctionalTest

class ClientLoginTest(FunctionalTest):
    def add_clients_for_user1(self):
        # Loguje się
        self.find_input_boxes_for_login()
        self.inputbox_name.send_keys('user')
        self.inputbox_password.send_keys('pass')
        self.login_button.click()
        self.wait_for(lambda: self.browser.find_element_by_link_text('Klienci').click())
        self.wait_for(lambda: self.browser.find_element_by_link_text('Dodaj klienta').click())

        self.send_form({'email': 'mail@ma.com',
                'name': 'Klient',
                'surname': 'Pierwszy',
                'description': 'opis',
                'phone_number': 123123123,
                'pin': '1111'
                        })

    def test_client_login(self):
        self.add_clients_for_user1()
        time.sleep(3)

