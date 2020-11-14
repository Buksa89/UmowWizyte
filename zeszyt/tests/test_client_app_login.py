from django.contrib.auth import authenticate
from django.contrib.auth.models import User
from django.test import TestCase
from unittest import skip
from ..forms import ClientLoginForm
from ..models import Client
from.base import BaseTest

class ClientLoginTemplateTests(BaseTest):
    def test_client_login_template(self):
        user = User.objects.create_user(username='user')
        response = self.client.get(f'/{user}/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_app/client_login.html')

    def test_client_login_template_post(self):
        user = User.objects.create_user(username='user')
        response = self.client.post(f'/{user}/', data={})
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_app/client_login.html')

    @skip
    def test_client_login_not_active_user(self):
        #TODO: Zablokuj formularz
        user = User.objects.create_user(username='user', is_active=False)
        response = self.client.get(f'/{user}/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_app/login_not_active.html')

    def test_client_redirect_to_dahboard_after_login(self):
        user = User.objects.create_user(username='user', password='pass')
        Client.objects.create(user=user, phone_number='111111111', pin='1234')
        response = self.client.post(f'/{user}/', data={'phone_number': '111111111', 'pin': '1234'})
        self.assertTemplateUsed(response, 'client_app/client_dashboard.html')

class ClientLoginFormTests(TestCase):
   def test_display_form(self):
        user = User.objects.create_user(username='user')
        response = self.client.get(f'/{user}/')
        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], ClientLoginForm)


class ClientLoginViewTests(TestCase):

    def test_client_login_display_errors(self):
        user = User.objects.create_user(username='user', password='pass')
        user2 = User.objects.create_user(username='user2', password='pass')
        client_ok = Client.objects.create(user=user, phone_number='111111111', pin='1234')
        client_not_active = Client.objects.create(user=user, phone_number='222222222', pin='1234', is_active=False)
        client_other_user = Client.objects.create(user=user2, phone_number='111111112', pin='1234')

        data_results = [{'data': {'phone_number': '111111111', 'pin': '1111'}, 'message': 'Dane nieprawidłowe'},
                        {'data': {'phone_number': '123123123', 'pin': '1234'}, 'message': 'Nie ma takiego numeru'},
                        {'data': {'phone_number': '111111112', 'pin': '1234'}, 'message': 'Nie ma takiego numeru'},
                        {'data': {'phone_number': '222222222', 'pin': '1234'}, 'message': 'Konto zablokowane'},
                        {'data': {'phone_number': '', 'pin': '222222222'}, 'message': 'Podaj numer telefonu'},
                        {'data': {'phone_number': '111111111', 'pin': ''}, 'message': 'Podaj pin'}]
        for data_result in data_results:
            response = self.client.post(f'/{user}/', data=data_result['data'])
            self.assertContains(response, data_result['message'])


    def test_incorrect_login_auhorize(self):
        user = User.objects.create_user(username='user', password='pass')
        client = Client.objects.create(user=user, phone_number='111111111', pin='1234')

        response = self.client.post(f'/{user}/', data={'phone_number': '111111111', 'pin': '1111'})
        response2 = self.client.post(f'/{user}/', data={'phone_number': '222222222', 'pin': '1234'})
        self.assertNotIn('client_authorized', self.client.session)

    def test_not_active_login_auhorize(self):
        user = User.objects.create_user(username='user', password='pass')
        Client.objects.create(user=user, phone_number='111111111', pin='1234', is_active=False)
        response = self.client.post(f'/{user}/', data={'phone_number': '111111111', 'pin': '1234'})
        self.assertNotIn('client_authorized', self.client.session)

    def test_correct_login_auhorize(self):
        user = User.objects.create_user(username='user', password='pass')
        Client.objects.create(user=user, phone_number='111111111', pin='1234')
        self.client.post(f'/{user}/', data={'phone_number': '111111111', 'pin': '1234'})
        self.assertIn('client_authorized', self.client.session)
        correct_session = {'phone':'111111111','user':'user'}
        self.assertEqual(self.client.session['client_authorized'], correct_session)

class ClientLoginRedirectsTests(BaseTest):
    @skip
    def test_panel_redirect_to_login_when_user_not_authorized(self):
        # TODO: Tutaj dodaj podstrowny panelu klienta
        user = User.objects.create_user(username='user', password='pass')
        client = Client.objects.create(user=user, phone_number='111111111', pin='1234')
        response = self.client.get(f'/{user}/ustawienia/')
        self.assertRedirects(response, f'/{user}/')

    def test_redirect_after_logout(self):
        user = User.objects.create_user(username='user', password='pass')
        client = Client.objects.create(user=user, phone_number='111111111', pin='1234')
        self.authorize_client(client.phone_number,user.username)
        response = self.client.get(f'/{user}/logout/')
        self.assertRedirects(response, f'/{user}/')


    def test_client_logged_to_user_but_not_logged_to_others(self):
        self.authorize_client()
        user2 = User.objects.create_user(username='user2', password='pass')
        response = self.client.get(f'/user2/')
        self.assertTemplateUsed(response, 'client_app/client_login.html')

    # TODO: Znajdz lepsze miejsce dla ponizszej metody
    def test_client_login_wrong_user(self):
        user = User.objects.create_user(username='user')
        response = self.client.post(f'/user2/', data={})
        self.assertNotEqual(response.status_code, 200)

