from unittest import skip
import time
from .base import FunctionalTest

class UserLoginTest(FunctionalTest):

    def test_login(self):
        """ Inicjalizacja danych logowania """
        wrong_login = 'wrong login'
        wrong_password = 'wrong_pass'
        user = self.users['ok1']
        user_not_active = self.users['not_active']

        """Wejście na stronę"""
        # Użytkownik wchodzi na stronę i widzi panel logowania
        self.browser.find_element_by_name('login-form')
        self.wait_for(lambda:self.assertTrue(self.browser.find_element_by_name('login-form')))

        """Logowanie z błędnym loginem"""
        # Loguje się błędnym loginem
        field = {'username':wrong_login,'password':user['password']}
        self.send_form(field)
        # Widzi powiadomienie, że login jest błędny
        self.wait_for(lambda: self.assertIn("Błędny login lub hasło",
                                            self.browser.find_element_by_tag_name('body').text))

        """Logowanie z błędnym hasłem"""
        # Loguje się błędnym hasłem
        field = {'username':user['username'],'password':wrong_password}
        self.send_form(field)
        # Widzi powiadomienie, że hasło jest błędne
        self.wait_for(lambda: self.assertIn("Błędny login lub hasło",
                                            self.browser.find_element_by_tag_name('body').text))

        """Logowanie zablokowanego użytkownika"""
        # Loguje się jako zablokowany użytkownik
        field = {'username':user_not_active['username'],'password':user_not_active['password']}
        self.send_form(field)
        # Widzi powiadomienie, że jego konto jest zablokowane
        self.wait_for(lambda: self.assertIn("Konto zablokowane",
                                            self.browser.find_element_by_tag_name('body').text))

        """Logowanie prawidłowego użytkownika"""
        # Loguje się prawidłowymi danymi
        field = {'username': user['username'], 'password': user['password']}
        self.send_form(field)
        # Wyświetla mu sie strona główna panelu administracyjnrgo
        self.wait_for(lambda: self.assertIn("Panel",
            self.browser.find_element_by_tag_name('body').text
        ))

        """Wylogowywanie"""
        # Klika "Wyloguj"
        self.wait_for(lambda: self.browser.find_element_by_link_text('Wyloguj').click())
        # Widzi ponownie stronę logowania
        self.wait_for(lambda:self.assertTrue(self.browser.find_element_by_name('login-form')))