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
         self.add_clients_for_user()
         correct_phone = '123123123'
         correct_pin = self.get_pin(correct_phone, 'user')
         # Klient wchodzi na stronę obcego użytkownika
         self.browser.get(self.get_new_url('login', 'user2'))
         time.sleep(0.5)
         # Widzi panel logowania
         self.wait_for(lambda: self.find_input_boxes_for_client_login())
         # Próbuje się zalogować
         self.wait_for(lambda: self.send_form({'phone_number':correct_phone, 'pin':correct_pin}))
         #Dostaje komunikat, że nie może
         self.wait_for(lambda: self.assertIn("Nie ma takiego numeru",
             self.browser.find_element_by_tag_name('body').text
         ))
         # Wchodzi na stronę właściwego użytkownika
         self.browser.get(self.get_new_url('user2', 'user'))
         time.sleep(0.5)
         # Próbuje się zalogować bez podania telefonu
         #TODO: Próba wpisania liter do telefonu
         self.wait_for(lambda: self.send_form({'phone_number': '', 'pin': correct_pin}))
         # dostaje komunikat, że musi wypełnić wszystkie pola
         self.wait_for(lambda: self.assertNotIn("Wyloguj",self.browser.find_element_by_tag_name('body').text))

         # Wpisuje numer telefonu, ale próbuje się zalogować bez pinu
         self.wait_for(lambda: self.send_form({'phone_number': correct_phone, 'pin': ''}))
         # dostaje komunikat, że musi wypełnić wszystkie pola
         self.wait_for(lambda: self.assertNotIn("Wyloguj", self.browser.find_element_by_tag_name('body').text))

         # wpisuje właściwy numer i pin
         self.wait_for(lambda: self.send_form({'phone_number': correct_phone, 'pin': correct_pin}))
         # Widzi panel klienta
         self.wait_for(lambda: self.assertIn("Wyloguj", self.browser.find_element_by_tag_name('body').text))

        # TODO: zablokowany użytkownik