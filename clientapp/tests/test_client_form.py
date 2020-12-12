from django.test import TestCase
from unittest import skip
from ..forms import AddVisitForm, ClientChooseVisitForm, ClientLoginForm
from userapp.tests.base import BaseTest

class LoginTests(BaseTest):

    def test_form_instance(self):
        """Czy formularz logowania wczytuje prawidłową instancję?"""

        user = self.create_user()
        response = self.client.get(f'/{user}/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], ClientLoginForm)


    def test_form_errors_display(self):
        """Czy formularz logowania prawidłowo wyświetla błędy?"""

        client = self.clients['active_1']
        client_full_data = self.clients_full_data['active_1']

        data_results = [{'data': {'phone_number': self.EMPTY, 'pin': client_full_data['pin']}, 'field':'phone_number', 'message': 'Podaj numer telefonu'},
                        {'data': {'phone_number': self.WRONG_DATA, 'pin': client_full_data['pin']}, 'field':'phone_number', 'message': 'Podaj prawidłowy numer telefonu'},
                        {'data': {'phone_number': client['phone_number'], 'pin': self.EMPTY}, 'field':'pin', 'message': 'Podaj pin'},
                        {'data': {'phone_number': client['phone_number'], 'pin': self.WRONG_DATA}, 'field':'pin', 'message': 'Podaj prawidłowy pin'},
                        ]

        for data_result in data_results:
            form = ClientLoginForm(data=data_result['data'])
            self.assertFalse(form.is_valid())
            self.assertEqual(form.errors[data_result['field']], [data_result['message']])


class DashboardTests(BaseTest):

    def test_form_instance(self):
        """Czy formularz dodawania wizyty wczytuje prawidłową instancję?"""
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/panel/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], ClientChooseVisitForm)


class AddVisitTests(BaseTest):

    def test_form_instance(self):
        """Czy formularz potwierdzania wizyty wczytuje prawidłową instancję?"""
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        date = self.weeks['no_holiday']
        self.authorize_client(client)
        date_time = self.date_time['regular']
        response = self.client.get(f"/{user}/nowa_wizyta/{service.id}/{date_time.year}/{date_time.month}/{date_time.day}/{date_time.hour}/{date_time.minute}/")
        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], AddVisitForm)

    @skip
    def test_form_errors_display(self):
        """Czy formularz potwierdzania wizyty prawidłowo wyświetla błędy?"""
        pass