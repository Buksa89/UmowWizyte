from django.contrib.auth.models import User
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from unittest import skip
import time
from .base import FunctionalTest


class PanelClientTest(FunctionalTest):

    def test_clients_add_remove(self):
        """ Inicjalizacja danych """
        client1 = self.clients['ok1']['cl_ok1']
        client2 = self.clients['ok1']['cl_ok2']
        user = 'ok1'
        self.login_user(user)

        """ Test okna "klienci" """
        # Wybiera z menu "Klienci"
        self.wait_for(lambda: self.browser.find_element_by_link_text('Klienci').click())
        # Dostaje wiadomość, że nie ma jeszcze żadnych klientów
        self.wait_for(lambda: self.assertIn("Nie masz jeszcze żadnych klientów",
            self.browser.find_element_by_tag_name('h3').text
        ))

        """ Test okna "Dodaj klienta" """
        # Klika w przycisk dodający klienta
        self.wait_for(lambda: self.browser.find_element_by_link_text('Dodaj klienta').click())
        self.wait_for(lambda: self.assertIn("Dodawanie klienta",
            self.browser.find_element_by_tag_name('h3').text
        ))


        """ Testy dodawania klienta """
        # Próbuje dodać klienta bez numeru telefonu
        fields = client1.copy()
        fields['phone_number'] = ''
        self.send_form(fields)
        self.wait_for(lambda:self.assertTrue(self.browser.find_element_by_name('add-client-form')))
        
        # Próbuje dodać klienta bez imienia
        fields = client1.copy()
        fields['name']=''
        self.send_form(fields)
        self.wait_for(lambda:self.assertTrue(self.browser.find_element_by_name('add-client-form')))

        # Dodaje pierwszego klienta
        self.send_form(client1)
        self.wait_for(lambda: self.assertIn(client1['name']+" dodany",
            self.browser.find_element_by_tag_name('body').text
        ))
        # Próbuje dodać klienta z tym samym numerem telefonu
        self.send_form(client1)
        self.wait_for(lambda: self.assertIn("Klient o podanym numerze telefonu już istnieje",
            self.browser.find_element_by_class_name('errorlist').text
        ))
        # Próbuje dodać klienta z błędnym numerem telefonu
        fields = client1.copy()
        fields['phone_number']='no_digit_phone'
        self.send_form(fields)
        self.wait_for(lambda: self.assertIn("Podaj prawidłowy numer telefonu",
            self.browser.find_element_by_class_name('errorlist').text
        ))
        # Dodaje drugiego klienta
        self.send_form(client2)
        self.wait_for(lambda: self.assertIn(client2['name']+" dodany",
            self.browser.find_element_by_tag_name('body').text
        ))
        # Przechodzi na listę klientów
        self.wait_for(lambda: self.browser.find_element_by_link_text('Kliknij tutaj').click())

        # TODO: Dodaj sortowanie listy klientów

        # Wyciągnięcie pinu z bazy
        client1['pin'] = self.get_pin(client1['phone_number'], self.users[user]['username'])
        client2['pin'] = self.get_pin(client2['phone_number'], self.users[user]['username'])

        # Widzi listę z dwoma pozycjami
        for field in client1.values():
            self.wait_for(lambda: self.assertIn(field, self.browser.find_elements_by_tag_name("tr")[1].text))
        for field in client2.values():
            self.wait_for(lambda: self.assertIn(field, self.browser.find_elements_by_tag_name("tr")[2].text))

        """ Testy usuwania klienta """
        # Usuwa klienta
        self.wait_for(lambda: self.browser.find_element_by_link_text('Usuń').click())
        for field in client1.values():
            self.wait_for(lambda: self.assertNotIn(field,
                                                self.browser.find_elements_by_tag_name("tr")[1].text
                                                ))
        for field in client2.values():
            self.wait_for(lambda: self.assertIn(field,
                                                self.browser.find_elements_by_tag_name("tr")[1].text
                                                ))


        #TODO: Czy inny user nie widzi tego klienta
        #TODO czy inny user może go usunąć
        #TODO czy inny user może dodać klienta o tym samym numerze



        # TODO: edycja klienta
        # TODO: inny user edytuje klienta
        # TODO: zapisanie zmian
        # TODO: próba usuniecia, ale bez potwierdzenia
        # TODO: Próba logowania klienta
        # TODO: zablokowanie klienta
        # TODO: Próba logowania na zablokowane konto


