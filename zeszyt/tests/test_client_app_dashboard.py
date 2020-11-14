from django.contrib.auth import authenticate
from django.contrib.auth.models import User
from django.test import TestCase
from unittest import skip
from ..forms import ClientChooseVisitForm
from ..models import Client
from.base import BaseTest

class ClientDashboardTemplateTests(BaseTest):
    def test_client_dashboard_template(self):
        self.authorize_client()
        response = self.client.get(f'/{self.user}/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_app/client_dashboard.html')

    def test_new_visit_template_post(self):
        self.authorize_client()
        response = self.client.post(f'/{self.user}/nowa_wizyta/', data={'service':'service'})
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_app/client_app_new_visit_step_1.html')

    def test_new_visit_template_no_post(self):
        self.authorize_client()
        response = self.client.get(f'/{self.user}/nowa_wizyta/')
        self.assertNotEqual(response.status_code, 200)

    """    @skip
    def test_client_login_not_active_user(self):
        #TODO: Zablokuj formularz
        user = User.objects.create_user(username='user', is_active=False)
        response = self.client.get(f'/{user}/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_app/login_not_active.html')

    def test_client_logged_to_panel(self):
        self.authorize_client()
        response = self.client.get(f'/user/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_app/client_dashboard.html')"""


class ClientDashboardFormTests(BaseTest):
   def test_display_form(self):
        self.authorize_client()
        response = self.client.get(f'/{self.user}/')
        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], ClientChooseVisitForm)




class ClientLoginViewTests(BaseTest):
    @skip #TODO: sprawdź czy wyświetlają się prawidłowe wiziyty
    def test_correct_visits_in_form(self):
        self.authorize_client()
        response = self.client.get(f'/{self.user}/')
        #print(response.content.decode())

"""

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
    @skip # moze bedzie zbedny
    def test_panel_redirect_to_login_when_user_not_authorized(self):
        # TODO: Tutaj dodaj podstrowny panelu klienta
        user = User.objects.create_user(username='user', password='pass')
        client = Client.objects.create(user=user, phone_number='111111111', pin='1234')
        response = self.client.get(f'/{user}/panel/')
        self.assertRedirects(response, f'/{user}/')

    def test_redirect_after_logout(self):
        user = User.objects.create_user(username='user', password='pass')
        client = Client.objects.create(user=user, phone_number='111111111', pin='1234')
        self.authorize_client(client.phone_number,user.username)
        response = self.client.get(f'/{user}/logout/')
        self.assertRedirects(response, f'/{user}/')
    @skip # TODO:  zbędne
    def test_redirect_login_to_panel_when_logged(self):
        user = User.objects.create_user(username='user', password='pass')
        client = Client.objects.create(user=user, phone_number='111111111', pin='1234')
        self.authorize_client(client.phone_number,user.username)
        response = self.client.get(f'/{user}/')
        self.assertRedirects(response, f'/{user}/panel/')

    @skip #TODO do przerobienia
    def test_client_logged_to_user_but_not_logged_to_others(self):
        self.authorize_client()
        user2 = User.objects.create_user(username='user2', password='pass')
        response = self.client.get(f'/user2/panel/')
        self.assertRedirects(response, f'/user2/')

    # TODO: Znajdz lepsze miejsce dla ponizszej metody
    def test_client_login_wrong_user(self):
        user = User.objects.create_user(username='user')
        response = self.client.post(f'/user2/', data={})
        self.assertNotEqual(response.status_code, 200)

"""