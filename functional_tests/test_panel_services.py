from unittest import skip
import time
from .base import FunctionalTest


class PanelServicesTest(FunctionalTest):

    def test_services_add_remove(self):
        """ Inicjalizacja danych """
        user = 'ok1'
        service1 = self.services[user]['serv_ok1']
        service2 = self.services[user]['serv_ok2']
        service_not_active = self.services[user]['serv_not_active']
        self.login_user(user)

        """ Test okna Ustawień """
        # Wybiera z menu "Ustawienia"
        self.wait_for(lambda: self.browser.find_element_by_link_text('Ustawienia').click())
        # Dostaje wiadomość, że nie ma jeszcze żadnych usług. Widzi też formularz dodwania usługi
        self.wait_for(lambda: self.assertIn("Nie masz jeszcze żadnych usług",
                                            self.browser.find_element_by_tag_name('h3').text
                                            ))
        self.wait_for(lambda: self.assertTrue(self.browser.find_element_by_name('add-service-form')))

        """ Testy dodawania usługi """
        # Próbuje dodać usługę bez nazwy
        fields = service1.copy()
        fields['name'] = ''
        self.wait_for(lambda: self.assertIn("Nie masz jeszcze żadnych usług",
                                            self.browser.find_element_by_tag_name('h3').text
                                            ))

        # Próbuje dodać usługę bez czasu
        fields = service1.copy()
        fields['duration'] = ''
        self.wait_for(lambda: self.assertIn("Nie masz jeszcze żadnych usług",
                                            self.browser.find_element_by_tag_name('h3').text
                                            ))

        # Próbuje dodać usługę bez oznaczania pola
        self.send_form(service_not_active)
        # Widzi, że usługa pojawiłą się na liście
        self.wait_for(lambda: self.assertIn(f"Usługa {service_not_active['name']} dodana",
                                            self.browser.find_element_by_tag_name('body').text
                                            ))

        for field in service_not_active.values():
            self.wait_for(lambda: self.assertIn(str(field), self.browser.find_elements_by_tag_name("tr")[1].text))
        # Dodaje kolejną usługę
        self.send_form(service1)
        # Na liście pojawiła się kolejna usługa
        self.wait_for(lambda: self.assertIn(f"Usługa {service1['name']} dodana",
                                            self.browser.find_element_by_tag_name('body').text
                                            ))
        for field in service1.values():
            self.wait_for(lambda: self.assertIn(str(field), self.browser.find_elements_by_tag_name("tr")[2].text))

        """ Testy usuwania usługi """
        # Usuwa klienta
        self.wait_for(lambda: self.browser.find_element_by_link_text('Usuń').click())
        for field in service1.values():
            self.wait_for(lambda: self.assertIn(str(field),
                                                self.browser.find_elements_by_tag_name("tr")[1].text
                                                ))
        for field, value in service_not_active.items():
            if field == 'duration':continue    #Jeśli duration jest takie samo s_n_a i s1, trzeba je wykluczyć
            self.wait_for(lambda: self.assertNotIn(str(value),
                                                self.browser.find_elements_by_tag_name("tr")[1].text
                                                ))

        # TODO: Testy usuwania usługi
        # TODO: edycja usługi
        # TODO: inny user edytuje usługę
        # TODO: zapisanie zmian
        # TODO: czy klient moze dodac zablokowaną usługę
        # TODO: Czy inne formularze się nie rozsypują



