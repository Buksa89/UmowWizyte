from datetime import timedelta
from django.contrib.auth.models import User
from django.test import TestCase
from unittest import skip
from .base import BaseTest
from ..models import Client, Service


# TODO: Czy klient widzi aktywną usługę?
# TODO: Czy klient widzi nieaktywną usługę?
# TODO: czy obcy klient widzi aktywną usługę

class DashboardClientsTests(BaseTest):

    def test_clients_form_display(self):
        self.authorize_user()
        response = self.client.get('/klienci/nowy/')

        self.assertContains(response, 'add-client-form')

    def test_other_user_see_my_clients(self):
        self.authorize_user()
        self.client.post('/klienci/nowy/', data={'pin':'1111', 'name':'aaa', 'phone_number':'12212121'})
        self.authorize_user('user2', 'pass2')
        response = self.client.get('/klienci/')

        self.assertContains(response, "Nie masz jeszcze żadnych klientów")

    # TODO Test widoku: Czy lista klientów się prawidłowo wyświetla
    # TODO Czy klient się dodaje po wysłąniu posta
    # TODO Czy klient się usuwa

    # TODO Czy obcy user widzi klientów


class DashboardTests(BaseTest):

    """ Client remove """

    def test_user_remove_client(self):
        self.authorize_user('user', 'pass')
        client = Client.objects.create(phone_number='111', user=self.user)
        self.client.get(client.get_remove_url())

        self.assertFalse(Client.objects.first())

    def test_user_cannot_remove_others_clients(self):
        """Użytkownik nie powinien mieć możliwości usunięcia klientów innego użytkownika
        poprzez podanie jego id w linku"""
        self.authorize_user('user', 'pass')
        client = Client.objects.create(phone_number='111', user=self.user)
        self.authorize_user('user2', 'pass2')
        self.client.get(client.get_remove_url())

        self.assertTrue(Client.objects.first())


class DashobardSettingsTests(BaseTest):

    """ Service add tests """

    def test_service_form_display(self):
        self.authorize_user()
        response = self.client.get('/ustawienia/')

        self.assertContains(response, 'add-service-form')

    def test_service_form_display_errors(self):
        name = "Muvaffakiyetsizleştiricilestiriveremeyebileceklerimizdenmissinizcesine"
        self.authorize_user()
        Service.objects.create(user=self.user, duration=timedelta(hours=1), name='duplikat')
        data_results = [{'data': {'duration': '00:15', 'name': ''}, 'message': 'Pole nie może być puste'},
                        {'data': {'duration': '00:00', 'name': 'usługa'}, 'message': 'Ustaw czas'},
                        {'data': {'duration': '01:45', 'name': name}, 'message': "Nazwa jest za długa"},
                        {'data': {'duration': '05:45', 'name': 'duplikat'}, 'message': 'Usługa o tej nazwie już istnieje'},]
        for data_result in data_results:
            response = self.client.post('/ustawienia/', data=data_result['data'])

            self.assertContains(response, data_result['message'])

    def test_service_empty_list_not_display(self):
        self.authorize_user()
        response = self.client.get('/ustawienia/')

        self.assertContains(response, "Nie masz jeszcze żadnych usług")

    def test_service_list_display(self):
        self.authorize_user()
        response = self.client.post('/ustawienia/', data={'duration': '00:15', 'name': 'usługa'})
        content = ['Usługa usługa dodana.', '<td>usługa</td>', '<td>00:15</td>', '<td>False</td>']

        for element in content:
            self.assertContains(response, element)

    def test_other_user_see_my_service(self):
        self.authorize_user()
        self.client.post('/ustawienia/', data={'duration': '00:15', 'name': 'usługa'})
        self.authorize_user('user2', 'pass2')
        response = self.client.get('/ustawienia/')

        self.assertContains(response, "Nie masz jeszcze żadnych usług")


    """ Service remove tests """

    def test_user_remove_service(self):
        self.authorize_user('user', 'pass')
        service = Service.objects.create(duration=timedelta(hours=1), name='usluga', user=self.user)
        self.client.get(service.get_remove_url())

        self.assertFalse(Service.objects.first())

    def test_user_cannot_remove_others_services(self):
        """ Jeden użytkownik nie powinien mieć możliwości usunięcia usługi innego, poprzez ręczne wpisanie
        linka z id usługi """
        self.authorize_user('user', 'pass')
        service = Service.objects.create(duration=timedelta(hours=1), name='usluga', user=self.user)
        self.authorize_user('user2', 'pass2')
        with self.assertRaises(Service.DoesNotExist):
            self.client.get(service.get_remove_url())
        self.assertTrue(Service.objects.first())


class UserLoginTests(TestCase):

    def test_incorrect_login_display_errors(self):
        User.objects.create_user(username='user', password='pass')
        User.objects.create_user(username='user2', password='pass2', is_active=False)
        data_results = [{'data': {'username':'user', 'password':'wrong_pass'}, 'message':'Błędny login lub hasło'},
                {'data': {'username':'wrong_username', 'password':'pass'}, 'message':'Błędny login lub hasło'},
                {'data': {'username':'user2', 'password':'pass2'}, 'message':'Konto zablokowane'},
                {'data': {'username':'', 'password':'pass2'}, 'message':'Podaj login'},
                {'data': {'username':'user2', 'password':''}, 'message':'Podaj hasło'}]

        for data_result in data_results:
            response = self.client.post('/login/', data=data_result['data'])
            self.assertContains(response, data_result['message'])

    def test_incorrect_login_auhorize(self):
        User.objects.create_user(username='user', password='pass')
        self.client.post('/login/', data={'username':'user', 'password':'wrong_pass'})
        self.client.post('/login/', data={'username':'wrong_username', 'password':'pass'})

        self.assertNotIn('_auth_user_id', self.client.session)

    def test_not_active_login_auhorize(self):
        User.objects.create_user(username='user', password='pass', is_active=False)
        self.client.post('/login/', data={'username':'user', 'password':'pass'})

        self.assertNotIn('_auth_user_id', self.client.session)

    def test_correct_login_auhorize(self):
        user = User.objects.create_user(username='user', password='pass')
        self.client.post('/login/', data={'username':'user', 'password':'pass'})

        self.assertIn('_auth_user_id', self.client.session)
        self.assertEqual(int(self.client.session['_auth_user_id']), user.pk)


    """ Login redirects tests """

    def test_panel_redirect_to_login_when_user_not_authorized(self):
        # TODO: Po dodaniu kazdej podstrony nalezy uzupelnic tą funkcję
        subpages = ['/panel/', '/klienci/', '/klienci/nowy/']

        for subpage in subpages:
            response = self.client.get(subpage)
            self.assertRedirects(response, f'/login/?next={subpage}')

    def test_redirect_after_logout(self):
        User.objects.create_user(username='testuser', password='12345')
        self.client.login(username='testuser', password='12345')
        response = self.client.get('/logout/')

        self.assertRedirects(response, f'/login/')

    def test_redirect_login_to_panel_when_logged(self):
        User.objects.create_user(username='testuser', password='12345')
        self.client.login(username='testuser', password='12345')
        response = self.client.get('/login/')

        self.assertRedirects(response, f'/panel/')