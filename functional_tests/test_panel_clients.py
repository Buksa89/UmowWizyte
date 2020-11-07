from django.contrib.auth.models import User
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from unittest import skip
import time
from .base import FunctionalTest

class PanelClientTest(FunctionalTest):

    def test_clients_panel(self):
        # Loguje się
        self.find_input_boxes_for_login()
        self.inputbox_name.send_keys('user')
        self.inputbox_password.send_keys('pass')
        self.login_button.click()
        # Wybiera z menu "Klienci"
        self.wait_for(lambda: self.browser.find_element_by_link_text('Klienci').click())
        # Dostaje wiadomość, że nie ma jeszcze żadnych klientów
        self.wait_for(lambda: self.assertIn("Nie masz jeszcze żadnych klientów",
            self.browser.find_element_by_tag_name('h3').text
        ))
        # Klika w przycisk dodający klienta
        self.wait_for(lambda: self.browser.find_element_by_link_text('Klienci').click())
        self.wait_for(lambda: self.assertIn("Nie masz jeszcze żadnych klientów",
            self.browser.find_element_by_tag_name('h3').text
        ))




        # TODO: dodanie klienta
        # TODO: dodanie klienta bez numeru telefonu
        # TODO: dodanie klienta o tym samym numerze telefonu
        # TODO: dodanie drugiego klienta
        # TODO: edycja klienta
        # TODO: zapisanie zmian
        # TODO: próba usuniecia, ale bez potwierdzenia
        # TODO: Próba logowania klienta
        # TODO: zablokowanie klienta
        # TODO: Próba logowania na zablokowane konto
        # TODO: usuniecie klienta