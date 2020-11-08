from django.contrib.auth.models import User
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from unittest import skip
import time
from .base import FunctionalTest

class PanelClientTest(FunctionalTest):

    def send_form(self, field):
        for field_name, field_value in field.items():
            self.browser.find_element_by_name(field_name).clear()
            self.browser.find_element_by_name(field_name).send_keys(field_value)
        self.browser.find_element_by_name('submit').click()

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
        self.wait_for(lambda: self.browser.find_element_by_link_text('Dodaj klienta').click())
        self.wait_for(lambda: self.assertIn("Dodawanie klienta",
            self.browser.find_element_by_tag_name('h3').text
        ))
        # Próbuje dodać klienta bez numeru telefonu
        self.send_form({'phone_number':'',
                        'email':'mail@ma.com',
                        'name':'no-phone',
                        'surname':'no-phone',
                        'description':'no-phone'})
        self.wait_for(lambda: self.assertNotIn("dodany",
            self.browser.find_element_by_tag_name('body').text
        ))
        # Próbuje dodać klienta bez imienia
        self.send_form({'phone_number':'',
                        'email':'mail@ma.com',
                        'name':'no-name',
                        'surname':'no-name',
                        'description':'no-name'})
        self.wait_for(lambda: self.assertNotIn("dodany",
            self.browser.find_element_by_tag_name('body').text
        ))
        # Dodaje pierwszego klienta
        self.send_form({'phone_number':'11111111',
                        'email':'mail@ma.com',
                        'name':'Pierwszy',
                        'surname':'klient',
                        'description':'opis pierwszego'})
        self.wait_for(lambda: self.assertIn("Pierwszy dodany",
            self.browser.find_element_by_tag_name('body').text
        ))
        # TODO: Próbuje dodać klienta z tym samym numerem telefonu
        self.send_form({'phone_number':'11111111',
                        'email':'mail@ma.com',
                        'name':'Duplikat',
                        'surname':'nDuplikat',
                        'description':'drugiego'})
        self.wait_for(lambda: self.assertNotIn("dodany",
            self.browser.find_element_by_tag_name('body').text
        ))
        time.sleep(3)

        # Dodaje drugiego klienta
        self.send_form({'phone_number':'222222222',
                        'email':'mail@ma.com',
                        'name':'Drugi',
                        'surname':'klient2',
                        'description':'opis drugiego'})
        self.wait_for(lambda: self.assertIn("Drugi dodany",
            self.browser.find_element_by_tag_name('body').text
        ))




        # TODO: dodanie drugiego klienta
        # TODO: dodanie klienta
        # TODO: dodanie klienta o tym samym numerze telefonu




        # TODO: edycja klienta
        # TODO: zapisanie zmian
        # TODO: próba usuniecia, ale bez potwierdzenia
        # TODO: Próba logowania klienta
        # TODO: zablokowanie klienta
        # TODO: Próba logowania na zablokowane konto
        # TODO: usuniecie klienta