from django.contrib.auth.models import User
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from unittest import skip
import time
from .base import FunctionalTest

class PanelClientTest(FunctionalTest):

    def get_client_data(self, client):
        data = {'email': client+'_mail@ma.com',
                'name': client+'_name',
                'surname': client+'_surname',
                'description': client+'_desc'}
        if client == 'no_phone': data['phone_number'] = ''
        elif client == 'no_name':
            data['phone_number'] = '123123123'
            data['name'] = ''
        elif client == 'first': data['phone_number'] = '111111111'
        elif client == 'duplicate': data['phone_number'] = '111111111'
        elif client == 'wrong_phone':  data['phone_number'] = 'aaa'
        elif client == 'second':  data['phone_number'] = '222222222'
        return data

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
        self.send_form(self.get_client_data('no_phone'))
        self.wait_for(lambda: self.assertNotIn("dodany",
            self.browser.find_element_by_tag_name('body').text
        ))
        # Próbuje dodać klienta bez imienia
        self.send_form(self.get_client_data('no_name'))
        self.wait_for(lambda: self.assertNotIn("dodany",
            self.browser.find_element_by_tag_name('body').text
        ))
        # Dodaje pierwszego klienta

        self.send_form(self.get_client_data('first'))
        self.wait_for(lambda: self.assertIn("first_name dodany",
            self.browser.find_element_by_tag_name('body').text
        ))
        # Próbuje dodać klienta z tym samym numerem telefonu
        self.send_form(self.get_client_data('duplicate'))
        self.wait_for(lambda: self.assertIn("Klient o podanym numerze telefonu już istnieje",
            self.browser.find_element_by_class_name('errorlist').text
        ))
        # Próbuje dodać klienta z błędnym numerem telefonu
        self.send_form(self.get_client_data('wrong_phone'))
        self.wait_for(lambda: self.assertIn("Podaj prawidłowy numer telefonu",
            self.browser.find_element_by_class_name('errorlist').text
        ))

        # Dodaje drugiego klienta
        self.send_form(self.get_client_data('second'))
        self.wait_for(lambda: self.assertIn("second_name dodany",
            self.browser.find_element_by_tag_name('body').text
        ))
        # Przechodzi na listę klientów
        self.wait_for(lambda: self.browser.find_element_by_link_text('Kliknij tutaj').click())

        # Widzi listę z dwoma pozycjami
        # TODO: Dodaj sortowanie listy klientów
        # TODO: Test czy pin się dodał
        for field in self.get_client_data('first').values():
            self.wait_for(lambda: self.assertIn(field,
                                                self.browser.find_elements_by_tag_name("tr")[1].text
                                                ))
        for field in self.get_client_data('second').values():
            self.wait_for(lambda: self.assertIn(field,
                                                self.browser.find_elements_by_tag_name("tr")[2].text
                                                ))

        #TODO: Czy inny user nie widzi tego klienta
        #TODO czy inny user może go usunąć



        # TODO: edycja klienta
        # TODO: inny user edytuje klienta
        # TODO: zapisanie zmian
        # TODO: próba usuniecia, ale bez potwierdzenia
        # TODO: Próba logowania klienta
        # TODO: zablokowanie klienta
        # TODO: Próba logowania na zablokowane konto

        # Usuwa klienta
        self.wait_for(lambda: self.browser.find_element_by_link_text('Usuń').click())
        for field in self.get_client_data('first').values():
            self.wait_for(lambda: self.assertNotIn(field,
                                                self.browser.find_elements_by_tag_name("tr")[1].text
                                                ))
        for field in self.get_client_data('second').values():
            self.wait_for(lambda: self.assertIn(field,
                                                self.browser.find_elements_by_tag_name("tr")[1].text
                                                ))


