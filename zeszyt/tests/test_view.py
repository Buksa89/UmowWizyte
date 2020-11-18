from datetime import timedelta, date
from django.contrib.auth.models import User
from django.test import TestCase
from unittest import skip
from .base import BaseTest
from ..models import Client, Service, WorkTime

#TODO: Testy formularza ustawiania czasu
#TODO: Testy wyświetlania nieaktywnych dni w kalendarzu



ANY_PIN = 9999
ANY_PHONE = 999999999
ANY_NAME = 'Gumiś'
# TODO: Czy klient widzi aktywną usługę?
# TODO: Czy klient widzi nieaktywną usługę?
# TODO: czy obcy klient widzi aktywną usługę

class ClientLoginTests(BaseTest):

    def test_client_login_display_errors(self):
        user = self.create_user('ok1')
        user2 = self.create_user('ok2')
        client_ok = self.create_client(user, 'cl_ok1')
        client_not_active = self.create_client(user, 'cl_not_active')
        client_other_user = self.create_client(user2, 'cl_ok1')
        data_results = [{'data': {'phone_number': client_ok.phone_number, 'pin': ANY_PIN}, 'message': 'Dane nieprawidłowe'},
                        {'data': {'phone_number': ANY_PHONE, 'pin': client_ok.pin}, 'message': 'Nie ma takiego numeru'},
                        {'data': {'phone_number': client_other_user.phone_number, 'pin': client_other_user.pin}, 'message': 'Nie ma takiego numeru'},
                        {'data': {'phone_number': client_not_active.phone_number, 'pin': client_not_active.pin}, 'message': 'Konto zablokowane'},
                        {'data': {'phone_number': '', 'pin': client_ok.pin}, 'message': 'Podaj numer telefonu'},
                        {'data': {'phone_number': client_ok.phone_number, 'pin': ''}, 'message': 'Podaj pin'}]
        for data_result in data_results:
            response = self.client.post(f'/{user}/', data=data_result['data'])
            self.assertContains(response, data_result['message'])

    def test_incorrect_login_auhorize(self):
        user = self.create_user()
        client = self.create_client(user)
        response = self.client.post(f'/{user}/', data={'phone_number': client.phone_number, 'pin': ANY_PIN})
        response2 = self.client.post(f'/{user}/', data={'phone_number': ANY_PHONE, 'pin': client.pin})
        self.assertNotIn('client_authorized', self.client.session)

    def test_not_active_login_auhorize(self):
        user = self.create_user()
        client = self.create_client(user, 'cl_not_active')
        response = self.client.post(f'/{user}/', data={'phone_number': client.phone_number, 'pin': client.pin})
        self.assertNotIn('client_authorized', self.client.session)

    def test_correct_login_auhorize(self):
        user = self.create_user()
        client = self.create_client(user)
        self.client.post(f'/{user}/', data={'phone_number': client.phone_number, 'pin': client.pin})
        self.assertIn('client_authorized', self.client.session)
        correct_session = {'phone':client.phone_number,'user':user.username}
        self.assertEqual(self.client.session['client_authorized'], correct_session)

    """ Redirects tests """


    def test_panel_redirect_to_login_when_user_not_authorized(self):
        pass
        # TODO: Tutaj dodaj podstrowny panelu klienta
        #user = self.create_user()
        #response = self.client.get(f'/{user}/#/')
        #self.assertRedirects(response, f'/{user}/')


    def test_redirect_after_logout(self):
        self.authorize_client()
        response = self.client.get(f'/{self.user}/logout/')
        self.assertRedirects(response, f'/{self.user}/')

    def test_client_logged_to_user_but_not_logged_to_others(self):
        self.authorize_client()
        user2 = self.create_user('ok2')
        response = self.client.get(f'/{user2}/')
        self.assertTemplateUsed(response, 'client_app/login.html')


class ClientDashboardTests(BaseTest):

    def test_correct_services_in_form(self):
        self.authorize_client()
        service1 = self.create_service(self.user, 'serv_ok1')
        service2 = self.create_service(self.user, 'serv_ok2')
        service3 = self.create_service(self.user, 'serv_not_active')
        response = self.client.get(f'/{self.user}/')
        self.assertContains(response, service1.name+'</option>')
        self.assertContains(response, service2.name+'</option>')
        self.assertNotContains(response, service3.name+'</option>')


class DasboardScheduleTests(BaseTest):

    def test_calendar_highlight_today(self):
        self.authorize_user()
        response = self.client.get('/terminarz/')

        self.assertContains(response, f'<span class="active">{date.today().strftime("%d")}</span>')

    def test_calendar_date_url(self):
        self.authorize_user()
        response = self.client.get('/terminarz/2011/9')

        self.assertContains(response, 'Wrzesień')

    def test_calendar_day_link(self):
        self.authorize_user()
        response = self.client.get('/terminarz/2011/9')

        self.assertContains(response, '<a href="/terminarz/2011/9/9"><li>9</li></a>')

    def test_calendar_holidays_red(self):
        self.authorize_user()
        response = self.client.get('/terminarz/2011/11')

        self.assertContains(response, '<li class="red">1</li>')
        self.assertContains(response, '<li class="red">11</li>')
        self.assertContains(response, '<li class="red">6</li>')
        self.assertNotContains(response, '<li class="red">2</li>')


class DashboardClientsTests(BaseTest):

    def test_clients_form_display(self):

        self.authorize_user()
        response = self.client.get('/klienci/nowy/')

        self.assertContains(response, 'add-client-form')

    def test_other_user_see_my_clients(self):
        user = self.create_user('ok1')
        self.create_client(user)
        self.authorize_user('ok2')
        response = self.client.get('/klienci/')

        self.assertContains(response, "Nie masz jeszcze żadnych klientów")

    def test_client_is_added_correctly(self):
        self.authorize_user()
        response = self.client.post('/klienci/nowy/', data={'pin':ANY_PIN, 'name':ANY_NAME, 'phone_number':ANY_PHONE})
        self.assertTrue(Client.objects.first())
        self.assertContains(response, ANY_NAME+" dodany")

    def test_clients_list_display(self):
        self.authorize_user('ok1')
        self.create_client(self.user,'cl_ok1')
        self.create_client(self.user,'cl_ok2')
        response = self.client.get('/klienci/')
        for field in self.clients['ok1']['cl_ok1'].values():
            self.assertContains(response, field)
        for field in self.clients['ok1']['cl_ok2'].values():
            self.assertContains(response, field)


class DashboardTests(BaseTest):

    """ Client remove """

    def test_user_remove_client(self):
        self.authorize_user()
        client = self.create_client(self.user)
        response = self.client.get(client.get_remove_url())
        self.assertFalse(Client.objects.first())
        self.assertRedirects(response, '/klienci/')

    def test_user_cannot_remove_others_clients(self):
        """Użytkownik nie powinien mieć możliwości usunięcia klientów innego użytkownika
        poprzez podanie jego id w linku"""
        user = self.create_user('ok1')
        client = self.create_client(user)
        self.authorize_user('ok2')
        self.client.get(client.get_remove_url())

        self.assertTrue(Client.objects.first())

#TODO: Refaktoryzacja testów poniżej:

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
        data_results = [{'data': {'duration': '00:15', 'name': '', 'submit': 'add_service'}, 'message': 'Pole nie może być puste'},
                        {'data': {'duration': '00:00', 'name': 'usługa', 'submit': 'add_service'}, 'message': 'Ustaw czas'},
                        {'data': {'duration': '01:45', 'name': name, 'submit': 'add_service'}, 'message': "Nazwa jest za długa"},
                        {'data': {'duration': '05:45', 'name': 'duplikat', 'submit': 'add_service'}, 'message': 'Usługa o tej nazwie już istnieje'}]
        for data_result in data_results:
            response = self.client.post('/ustawienia/', data=data_result['data'])

            self.assertContains(response, data_result['message'])

    def test_service_empty_list_not_display(self):
        self.authorize_user()
        response = self.client.get('/ustawienia/')

        self.assertContains(response, "Nie masz jeszcze żadnych usług")

    def test_service_list_display(self):
        self.authorize_user()
        response = self.client.post('/ustawienia/', data={'duration': '00:15', 'name': 'usługa', 'submit': 'add_service'})
        content = ['Usługa usługa dodana.', '<td>usługa</td>', '<td>00:15</td>', '<td>False</td>']
        for element in content:
            self.assertContains(response, element)

    def test_other_user_see_my_service(self):
        self.authorize_user('ok1')
        self.client.post('/ustawienia/', data={'duration': '00:15', 'name': 'usługa', 'submit': 'add_service'})
        self.authorize_user('ok2')
        response = self.client.get('/ustawienia/')

        self.assertContains(response, "Nie masz jeszcze żadnych usług")


    """ Service remove tests """

    def test_user_remove_service(self):
        self.authorize_user()
        service = Service.objects.create(duration=timedelta(hours=1), name='usluga', user=self.user)
        self.client.get(service.get_remove_url())

        self.assertFalse(Service.objects.first())

    def test_user_cannot_remove_others_services(self):
        """ Jeden użytkownik nie powinien mieć możliwości usunięcia usługi innego, poprzez ręczne wpisanie
        linka z id usługi """
        self.authorize_user('ok1')
        service = Service.objects.create(duration=timedelta(hours=1), name='usluga', user=self.user)
        self.authorize_user('ok2')
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