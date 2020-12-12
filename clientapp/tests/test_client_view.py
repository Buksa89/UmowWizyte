from bs4 import BeautifulSoup as Bs
from datetime import date, datetime, timedelta
from django.test import TestCase
from unittest import skip
from ..forms import ClientLoginForm
from userapp.models import WorkTime
from userapp.tests.base import BaseTest


class DashboardTests(BaseTest):

    def test_form_display(self):
        """Czy formularz dodawania wizyty się wyświetla? """
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/panel/')
        soup = Bs(response.content.decode(), features="html.parser")
        form = soup.find("form", {"name":"choose-visit-form"})

        self.assertTrue(form)

    def test_correct_services_in_form(self):
        """Czy klient widzi prawidłowe usługi (swojego użytkownika, niezablokowane)"""
        user_correct = self.create_user('active_1')
        user_wrong = self.create_user('active_2')

        service_wrong = self.create_service(user_wrong, 'long_2')
        service_not_active = self.create_service(user_correct, 'not_active')
        services_correct = []
        services_types = ['short_1', 'short_2', 'long_1']
        for service in services_types:
            services_correct.append(self.create_service(user_correct, service))

        client = self.create_client(user_correct)
        self.authorize_client(client)
        response = self.client.get(f'/{user_correct}/panel/')
        soup = Bs(response.content.decode(), features="html.parser")
        option_wrong = soup.find("option", {"value": {service_wrong.id}})
        option_not_active = soup.find("option", {"value": {service_not_active.id}})

        self.assertFalse(option_wrong)
        self.assertFalse(option_not_active)

        for service in services_correct:
            options = soup.find("option", {"value": {service.id}})

            self.assertEqual(options.get_text(), service.name)


    def test_empty_visits_lists(self):
        """Czy wyświetla się powiadomienie o braku nadchodzących wizyt"""
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/panel/')

        self.assertContains(response, 'Nie jesteś umówiony na żadną wizytę')


    def test_visits_lists(self):
        """Czy lista wizyt prawidłowo się wyświetla (wizyty dn dp on op, brak wizyt przedawnionych i wizyt innych klientów)"""
        user = self.create_user()
        correct_client = self.create_client(user, 'active_1')
        wrong_client = self.create_client(user, 'active_2')
        self.authorize_client(correct_client)

        # Not display
        old_visit = self.create_visit(user, correct_client, 'old')
        wrong_visit = self.create_visit(user, wrong_client, 'confirmed_1')
        cancelled_confirmed = self.create_visit(user, correct_client, 'cancelled_confirmed')

        #display
        not_confirmed = self.create_visit(user, correct_client, 'not_confirmed') # display cancel button
        confirmed = self.create_visit(user, correct_client, 'confirmed_2') # display cancel button
        cancelled_not_confirmed = self.create_visit(user, correct_client, 'cancelled_not_confirmed') # not display cancel button

        response = self.client.get(f'/{user}/panel/')
        self.assertContains(response, not_confirmed.name)
        self.assertContains(response, confirmed.name)
        self.assertContains(response, cancelled_not_confirmed.name)
        self.assertNotContains(response, old_visit.name)
        self.assertNotContains(response, wrong_visit.name)
        self.assertNotContains(response, cancelled_confirmed.name)

    def test_wrong_visit_cancel(self):
        """Czy klient może ręcznie odwołać cudzą, minioną, odwołaną wizytę (404)"""
        user = self.create_user()
        correct_client = self.create_client(user, 'active_1')
        wrong_client = self.create_client(user, 'active_2')
        self.authorize_client(correct_client)
        visits = []
        visits.append(self.create_visit(user, correct_client, 'old'))
        visits.append(self.create_visit(user, wrong_client, 'confirmed_1'))
        visits.append(self.create_visit(user, correct_client, 'cancelled_confirmed'))

        for visit in visits:
            response = self.client.get(f'/{user}/odwolaj/{visit.id}/')
            self.assertEqual(response.status_code, 404)

    def test_dashboard_POST_redirect(self):
        """Czy dashboard prawidłowo przekierowuje po otrzymaniu POST"""

        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        service = self.create_service(user)
        response = self.client.post(f'/{user}/panel/', data={'service': service.id})

        self.assertRedirects(response, f'/{user}/nowa_wizyta/{service.id}/')


class AddVisitTests(BaseTest):
    pass




class LoginTests(BaseTest):

    def test_incorrect_data_errors_display(self):
        """Czy błędy w formularzu logowania prawidłowo się wyświetlają"""
        user_active_1 = self.create_user('active_1')
        user_active_2 = self.create_user('active_2')

        client_of_user_1 = self.create_client(user_active_1, 'active_1')
        client_not_active = self.create_client(user_active_1, 'not_active')
        client_of_user_2 = self.create_client(user_active_2, 'active_2')

        data_results = [{'data': {'phone_number': client_of_user_1.phone_number, 'pin': self.ANY_PIN}, 'message': 'Dane nieprawidłowe'},
                        {'data': {'phone_number': self.ANY_PHONE, 'pin': client_of_user_1.pin}, 'message': 'Nie ma takiego numeru'},
                        {'data': {'phone_number': client_of_user_2.phone_number, 'pin': client_of_user_2.pin}, 'message': 'Nie ma takiego numeru'},
                        {'data': {'phone_number': client_not_active.phone_number, 'pin': client_not_active.pin}, 'message': 'Konto zablokowane'},
                        {'data': {'phone_number': self.EMPTY, 'pin': client_of_user_1.pin}, 'message': 'Podaj numer telefonu'},
                        {'data': {'phone_number': client_of_user_1.phone_number, 'pin': self.EMPTY}, 'message': 'Podaj pin'},
                        {'data': {'phone_number': client_of_user_1.phone_number, 'pin': self.WRONG_DATA}, 'message': 'Podaj prawidłowy pin'},
                        {'data': {'phone_number': self.WRONG_DATA, 'pin': client_of_user_1.pin}, 'message': 'Podaj prawidłowy numer telefonu'}]

        for data_result in data_results:
            response = self.client.post(f'/{user_active_1}/', data=data_result['data'])
            soup = Bs(response.content.decode(), features="html.parser")
            error_message = [td.find('li') for td in soup.findAll("ul", {"class": "errorlist"})]
            error_message = error_message[0].getText()

            self.assertIn(data_result['message'], error_message)


    def test_incorrect_data_not_authorize(self):
        """Czy klient pozostaje niezalogowany, jeśli loguje się z błędnymi danymi?"""
        user = self.create_user()
        client = self.create_client(user)
        self.client.post(f'/{user}/', data={'phone_number': client.phone_number, 'pin': self.ANY_PIN})
        self.client.post(f'/{user}/', data={'phone_number': self.ANY_PHONE, 'pin': client.pin})

        self.assertNotIn('client_authorized', self.client.session)


    def test_not_active_client_not_auhorize(self):
        """Czy klient pozostaje niezalogowany, jeśli jest nieaktywny?"""
        user = self.create_user()
        client = self.create_client(user, 'not_active')
        self.client.post(f'/{user}/', data={'phone_number': client.phone_number, 'pin': client.pin})

        self.assertNotIn('client_authorized', self.client.session)


    def test_correct_client_auhorized(self):
        """Czy klient zostaje zalogowany, jeśli loguje się z prawidłowymi danymi?"""
        user = self.create_user()
        client = self.create_client(user)
        self.client.post(f'/{user}/', data={'phone_number': client.phone_number, 'pin': client.pin})
        self.assertIn('client_authorized', self.client.session)
        correct_session = {'phone': client.phone_number, 'user': user.username}
        self.assertEqual(self.client.session['client_authorized'], correct_session)\


    def test_remove_client_from_session_after_logout(self):
        """Czy po wylogowaniu klient zostaje usunięty z sesji"""
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        self.client.get(f'/{user}/logout/')
        self.assertNotIn('client_authorized', self.client.session)


    """ Redirects tests """

    def test_redirect_to_login_url_when_client_not_authorized_for_this_user(self):
        """Czy niezalogowany klient zostaje przekierowany z podstron do strony logowania?"""
        user = self.create_user('active_1')
        user2 = self.create_user('active_2')
        client = self.create_client(user2)
        self.authorize_client(client)

        # TODO: Po dodaniu kazdej podstrony nalezy uzupelnic tą funkcję
        subpages = ['panel']

        for subpage in subpages:
            response = self.client.get(f'/{user}/{subpage}/')

            self.assertRedirects(response, f'/{user}/')


    def test_redirect_from_login_to_dashboard_url_when_client_authorized_for_this_user(self):
        """Czy zalogowany klient zostaje przekierowany ze strony logowania do pulpitu?"""

        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/')

        self.assertRedirects(response, f'/{user}/panel/')


    def test_redirect_to_dashboard_after_login(self):
        """Czy klient, który się właśnie zalogował, zostaje przekierowany do pulpitu?"""

        user = self.create_user()
        client = self.create_client(user)
        response = self.client.post(f'/{user}/', data={'phone_number':client.phone_number, 'pin':client.pin})

        self.assertRedirects(response, f'/{user}/panel/')


    def test_redirect_after_logout(self):
        """Czy klient zostaje przekierowany do strony logowania po wylogowaniu?"""

        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/logout/')

        self.assertRedirects(response, f'/{user}/')