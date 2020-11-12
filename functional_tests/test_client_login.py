from django.contrib.auth.models import User
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from unittest import skip
import time
from .base import FunctionalTest
from zeszyt.models import Client

class ClientLoginTest(FunctionalTest):

    def test_client_login(self):
        """ Inicjalizacja danych """
        self.add_some_clients()
        client = self.clients['ok1']['cl_ok1']

        """ Próba zalogowania na niewłaściwy terminarz """
        # Klient wchodzi na stronę obcego użytkownika
        self.browser.get(self.get_new_url('login', self.users['ok2']['username']))
        time.sleep(0.5)
        # Widzi panel logowania
        self.wait_for(lambda:self.assertTrue(self.browser.find_element_by_name('client-login-form')))
        # Próbuje się zalogować
        self.wait_for(lambda: self.send_form({'phone_number':client['phone_number'], 'pin':client['pin']}))
        #Dostaje komunikat, że nie może
        self.wait_for(lambda: self.assertIn("Nie ma takiego numeru",
            self.browser.find_element_by_tag_name('body').text
        ))

        """ Przejście na właściwy terminarz """
        self.browser.get(self.get_new_url(self.users['ok2']['username'], self.users['ok1']['username']))
        time.sleep(0.5)

        """Próba logowania na pusty numer telefonu"""
        #TODO: Próba wpisania liter do telefonu, pinu, złego telefonu, złego pinu, zablokowanego uzytkownika
        self.wait_for(lambda: self.send_form({'phone_number': '', 'pin': client['pin']}))
        # dostaje komunikat, że musi wypełnić wszystkie pola
        self.wait_for(lambda:self.assertTrue(self.browser.find_element_by_name('client-login-form')))

        """ Próba logowania bez pinu """
        self.wait_for(lambda: self.send_form({'phone_number': client['phone_number'], 'pin': ''}))
        # dostaje komunikat, że musi wypełnić wszystkie pola
        self.wait_for(lambda:self.assertTrue(self.browser.find_element_by_name('client-login-form')))

        """ Logowanie """
        self.wait_for(lambda: self.send_form({'phone_number': client['phone_number'], 'pin': client['pin']}))
        # Widzi panel klienta
        self.wait_for(lambda: self.assertIn("Wyloguj", self.browser.find_element_by_tag_name('body').text))

        """ Wylogowanie """
        self.wait_for(lambda: self.browser.find_element_by_link_text("Wyloguj").click())
        # Widzi panel logowania
        self.wait_for(lambda:self.assertTrue(self.browser.find_element_by_name('client-login-form')))