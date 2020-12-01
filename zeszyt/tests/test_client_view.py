from django.test import TestCase
from unittest import skip
from ..forms import ClientLoginForm
from .base import BaseTest

class LoginTests(BaseTest):

    def test_incorrect_data_errors_display(self):
        #TODO: Testy za długiego pinu,
        # za długiego numeru
        # pin not digits
        # tel_not_digits
        #TODO Anyphone i anypin ze zmiennej

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
                        {'data': {'phone_number': client_of_user_1.phone_number, 'pin': self.EMPTY}, 'message': 'Podaj pin'}]

        for data_result in data_results:
            response = self.client.post(f'/{user_active_1}/', data=data_result['data'])

            self.assertContains(response, data_result['message'])

    def test_incorrect_data_not_authorize(self):
        user = self.create_user()
        client = self.create_client(user)
        self.client.post(f'/{user}/', data={'phone_number': client.phone_number, 'pin': self.ANY_PIN})
        self.client.post(f'/{user}/', data={'phone_number': self.ANY_PHONE, 'pin': client.pin})

        self.assertNotIn('client_authorized', self.client.session)


    def test_not_active_client_not_auhorize(self):
        user = self.create_user()
        client = self.create_client(user, 'not_active')
        self.client.post(f'/{user}/', data={'phone_number': client.phone_number, 'pin': client.pin})

        self.assertNotIn('client_authorized', self.client.session)


    def test_correct_client_auhorized(self):
        user = self.create_user()
        client = self.create_client(user)
        self.client.post(f'/{user}/', data={'phone_number': client.phone_number, 'pin': client.pin})
        self.assertIn('client_authorized', self.client.session)
        correct_session = {'phone':client.phone_number,'user':user.username}
        self.assertEqual(self.client.session['client_authorized'], correct_session)\


    def test_remove_client_from_session_after_logout(self):
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/logout/')
        self.assertNotIn('client_authorized', self.client.session)


    """ Redirects tests """

    def test_redirect_to_login_url_when_client_not_authorized_for_this_user(self):
        # TODO: Tutaj dodaj podstrowny panelu klienta
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
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/')

        self.assertRedirects(response, f'/{user}/panel/')


    def test_redirect_to_dashboard_after_login(self):
        user = self.create_user()
        client = self.create_client(user)
        response = self.client.post(f'/{user}/', data={'phone_number':client.phone_number, 'pin':client.pin})

        self.assertRedirects(response, f'/{user}/panel/')


    def test_redirect_after_logout(self):
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/logout/')

        self.assertRedirects(response, f'/{user}/')