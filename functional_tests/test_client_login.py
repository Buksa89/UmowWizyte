from django.contrib.auth.models import User
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from unittest import skip
import time
from .base import FunctionalTest
from zeszyt.models import Client

class ClientLoginTest(FunctionalTest):
    def add_clients_for_user1(self):
        # Loguje się
        self.find_input_boxes_for_login()
        self.inputbox_name.send_keys('user')
        self.inputbox_password.send_keys('pass')
        self.login_button.click()
        self.wait_for(lambda: self.browser.find_element_by_link_text('Klienci').click())
        self.wait_for(lambda: self.browser.find_element_by_link_text('Dodaj klienta').click())

        self.wait_for(lambda: self.send_form({'email': 'mail@ma.com',
                'name': 'Klient',
                'surname': 'Pierwszy',
                'description': 'opis',
                'phone_number': '123123123',
                        }))
        self.wait_for(lambda: self.browser.find_element_by_link_text('Wyloguj').click())

    def get_pin(self, number, user):
        user = User.objects.get(username=user)
        return Client.objects.get(phone_number=number, user=user).pin

    def get_new_url(self, old, new):
         return self.browser.current_url.replace(old, new)

    def test_client_login(self):
         # Utworzenie klienta
         self.add_clients_for_user1()
         # Klient wchodzi na stronę obcego użytkownika
         #self.browser.get(self.get_new_url('login', 'user2'))
         # Widzi panel logowania
         #self.find_input_boxes_for_client_login()
         # Próbuje się zalogować
         #self.input_phone.send_keys('123123123')
         #self.input_pin.send_keys('pass')
         #self.submit_btn.click()
         #print(self.get_pin('123123123', 'user'))
         # Dostaje komunikat, że nie może
         #self.wait_for(lambda: self.assertIn("Nie ma takiego numeru",
         #    self.browser.find_element_by_tag_name('body').text
         #))
         # Wchodzi na stronę właściwego użytkownika
         #self.browser.get(self.live_server_url+'/user2')

         # Próbuje się zalogować

        # dostaje komunikat, że musi wypełnić wszystkie pola

        # Wpisuje numer telefonu, ale próbuje się zalogować bez pinu

        # dostaje komunikat, że musi wypełnić wszystkie pola

        # wpisuje właściwy numer i pin

        # Widzi panel klienta

        # TODO: zablokowany użytkownik