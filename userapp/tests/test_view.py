from django.test import TestCase
from unittest import skip
from ..models import Client, Service, WorkTime
from .base import BaseTest

class DashboardClientsTests(BaseTest):

    """ Clients list"""

    def test_empty_clients_list_display(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.get('/klienci/')

        self.assertContains(response, 'Nie masz jeszcze żadnych klientów')


    def test_clients_list_display(self):
        user = self.create_user()
        self.authorize_user(user)
        self.create_client(user,'active_1')
        self.create_client(user,'active_2')
        response = self.client.get('/klienci/')

        for field in self.clients['active_1'].values():
            self.assertContains(response, field)
        for field in self.clients['active_2'].values():
            self.assertContains(response, field)


    def test_other_user_sees_my_clients(self):
        user = self.create_user('active_1')
        user2 = self.create_user('active_2')
        self.create_client(user,'active_1')
        self.authorize_user(user2)
        response = self.client.get('/klienci/')

        self.assertContains(response, "Nie masz jeszcze żadnych klientów")


    """ Client add """

    def test_add_client_form_display(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.get('/klienci/nowy/')

        self.assertContains(response, 'add-client-form')


    def test_add_client_correctly(self):
        user = self.create_user()
        self.authorize_user(user)
        client = self.clients['active_1']
        client_full_data = self.clients_full_data['active_1']
        response = self.client.post('/klienci/nowy/', data={'pin':client_full_data['pin'], 'name':client['name'],
                                                            'phone_number':client['phone_number']})

        self.assertTrue(Client.objects.first())
        self.assertContains(response, client['name']+" dodany")


    """ Client remove """

    def test_remove_client_correctly(self):
        user = self.create_user()
        self.authorize_user(user)
        client = self.create_client(user)
        response = self.client.get(client.get_remove_url())

        self.assertFalse(Client.objects.first())
        self.assertRedirects(response, '/klienci/')


    def test_user_cannot_remove_others_clients(self):
        """ User should remove other user's clients by manual url """
        user = self.create_user('active_1')
        user2 = self.create_user('active_2')
        client = self.create_client(user)
        self.authorize_user(user2)
        response = self.client.get(client.get_remove_url())

        self.assertEqual(response.status_code, 404)
        self.assertTrue(Client.objects.first())


class DashboardSettingsTests(BaseTest):

    """ Service add tests """

    def test_service_form_display(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.get('/ustawienia/')

        self.assertContains(response, 'add-service-form')


    #todo: Ustaw czas do poprawy
    def test_service_form_display_errors(self):
        user = self.create_user()
        self.authorize_user(user)
        self.create_service(user, 'short_1')
        service = self.services['short_1']
        data_results = [{'data': {'duration': service['duration'], 'name': self.EMPTY, 'submit': 'add_service'}, 'message': 'Pole nie może być puste'},
                       # {'data': {'duration': self.DURATION_MIN, 'name': service['name'], 'submit': 'add_service'}, 'message': 'Ustaw czas'},
                        {'data': {'duration': service['duration'], 'name': self.LONG_STRING(61), 'submit': 'add_service'}, 'message': "Nazwa jest za długa"},
                        {'data': {'duration': service['duration'], 'name': service['name'], 'submit': 'add_service'}, 'message': 'Usługa o tej nazwie już istnieje'}]
        for data_result in data_results:
            response = self.client.post('/ustawienia/', data=data_result['data'])

            self.assertContains(response, data_result['message'])

    def test_service_empty_list_not_display(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.get('/ustawienia/')

        self.assertContains(response, "Nie masz jeszcze żadnych usług")

    def test_service_list_display(self):
        user = self.create_user()
        self.authorize_user(user)
        service = self.services['short_1']
        response = self.client.post('/ustawienia/', data={'duration': service['duration'], 'name': service['name'], 'is_active':service['is_active'], 'submit': 'add_service'})
        service['duration'] = str(service['duration'])[:-3].rjust(5,'0')
        content = [f"Usługa {service['name']} dodana.", f"<td>{service['duration']}</td>", f"<td>{service['name']}</td>", f"<td>Aktywna</td>"]

        for element in content:
            self.assertContains(response, element)

    def test_other_user_see_my_service(self):
        user1 = self.create_user('active_1')
        user2 = self.create_user('active_2')
        self.create_service(user1, 'short_1')
        self.authorize_user(user2)

        response = self.client.get('/ustawienia/')

        self.assertContains(response, "Nie masz jeszcze żadnych usług")


    """ Service remove tests """

    def test_remove_service(self):
        user = self.create_user()
        service = self.create_service(user)
        self.authorize_user(user)
        self.client.get(service.get_remove_url())

        self.assertFalse(Service.objects.first())


    def test_cannot_remove_others_services(self):
        """ Jeden użytkownik nie powinien mieć możliwości usunięcia usługi innego, poprzez ręczne wpisanie
        linka z id usługi """

        user = self.create_user('active_1')
        user2 = self.create_user('active_2')
        service = self.create_service(user)
        self.authorize_user(user2)
        response = self.client.get(service.get_remove_url())

        self.assertEqual(response.status_code, 404)
        self.assertTrue(Service.objects.first())


    """ Work time Tests """

    def test_work_time_form_display(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.get('/ustawienia/')

        self.assertContains(response, 'work-time-form')


    def test_work_time_form_working(self):
        user = self.create_user()
        self.authorize_user(user)
        data = {'start_time': '6:00', 'end_time': '8:00', 'monday': False, 'tuesday': True, 'wednesday': False,
                'thursday': True, 'friday': False, 'saturday': True, 'sunday': False, 'holidays': True,
                'earliest_visit': 1, 'latest_visit': 7, 'submit': 'set_work_time'}

        response = self.client.post('/ustawienia/', data = data)
        self.assertContains(response, 'Czas pracy zmieniony')
        self.assertContains(response, 'monday">')
        self.assertContains(response, 'tuesday" checked>')
        self.assertContains(response, 'wednesday">')
        self.assertContains(response, 'thursday" checked>')
        self.assertContains(response, 'friday">')
        self.assertContains(response, 'saturday" checked>')
        self.assertContains(response, 'sunday">')
        self.assertContains(response, 'holidays" checked>')
        self.assertContains(response, '"earliest_visit" value="1"')
        self.assertContains(response, 'latest_visit" value="7"')


    def test_service_form_errors(self):
        user = self.create_user()
        self.authorize_user(user)
        data = self.work_time
        error_visits = []
        error_data = [{'data':{'start_time': '10:00', 'end_time': '8:00', },'message':'Popraw godziny pracy'},
                      {'data':{'earliest_visit': 7, 'latest_visit': 2, },'message':'Popraw możliwość wyboru terminów'},
                      #TODO: Spolszczenie tego błędu
                      {'data':{'earliest_visit': -7, 'latest_visit': -2},'message':'Ensure this value is greater than or equal to 0.'}
                     ]
        for error_element in error_data:
            error_visit = {}
            error_visit['data'] = data.copy()
            error_visit['message'] = error_element['message']
            for field, value in error_element['data'].items():
                error_visit['data'][field] = value
            error_visits.append(error_visit)

        for error_visit in error_visits:
            response = self.client.post('/ustawienia/', data=error_visit['data'])
            self.assertContains(response, error_visit['message'])


class LoginTests(BaseTest):

    def test_incorrect_data_errors_display(self):
        user = self.create_user('active_1')
        user.password = self.users['active_1']['password']
        user_not_active = self.create_user('not_active')
        data_results = [{'data': {'username': user.username, 'password': self.WRONG_DATA}, 'message': 'Błędny login lub hasło'},
                        {'data': {'username': self.WRONG_DATA, 'password': user.password}, 'message': 'Błędny login lub hasło'},
                        {'data': {'username': user_not_active.username, 'password': user_not_active.password}, 'message': 'Konto zablokowane'},
                        {'data': {'username': self.EMPTY, 'password': user.password}, 'message': 'Podaj login'},
                        {'data': {'username': user.username, 'password': self.EMPTY}, 'message': 'Podaj hasło'}]

        for data_result in data_results:
            response = self.client.post('/login/', data=data_result['data'])

            self.assertContains(response, data_result['message'])


    def test_incorrect_data_not_authorize(self):
        user = self.create_user('active_1')
        user.password = self.users['active_1']['password']
        self.client.post('/login/', data={'username': user.username, 'password': self.WRONG_DATA})
        self.client.post('/login/', data={'username': self.WRONG_DATA, 'password': user.password})

        self.assertNotIn('_auth_user_id', self.client.session)


    def test_not_active_user_not_auhorize(self):
        user = self.create_user('not_active')
        user.password = self.users['not_active']['password']
        self.client.post('/login/', data={'username': user.username, 'password':user.password})

        self.assertNotIn('_auth_user_id', self.client.session)


    def test_correct_user_auhorized(self):
        user = self.create_user('active_1')
        user.password = self.users['active_1']['password']
        self.client.post('/login/', data={'username':user.username, 'password':user.password})

        self.assertIn('_auth_user_id', self.client.session)
        self.assertEqual(int(self.client.session['_auth_user_id']), user.pk)


    def test_remove_user_from_session_after_logout(self):
        user = self.create_user('active_1')
        user.password = self.users['active_1']['password']
        self.authorize_user(user)
        self.client.get('/logout/')

        self.assertNotIn('_auth_user_id', self.client.session)


    """ Redirects """

    def test_redirect_to_login_url_when_user_not_authorized(self):
        # TODO: Po dodaniu kazdej podstrony nalezy uzupelnic tą funkcję
        subpages = ['panel', 'klienci', 'klienci/nowy', 'terminarz', 'ustawienia']

        for subpage in subpages:
            response = self.client.get(f'/{subpage}/')

            self.assertRedirects(response, f'/login/?next=/{subpage}/')


    def test_redirect_from_login_to_dashboard_url_when_user_authorized(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.get('/login/')

        self.assertRedirects(response, f'/panel/')


    def test_user_redirect_to_dashboard_after_POST_login(self):
        user = self.create_user()
        user.password = self.users['active_1']['password']
        response = self.client.post('/login/', data={'username':user.username, 'password':user.password})

        self.assertRedirects(response, f'/panel/')


    def test_redirect_after_logout(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.get('/logout/')

        self.assertRedirects(response, f'/login/')
