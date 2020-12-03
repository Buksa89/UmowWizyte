from django.test import TestCase
from unittest import skip
from ..forms import ClientChooseVisitForm, ClientLoginForm
from zeszyt.tests.base import BaseTest

class LoginTests(BaseTest):


    def test_form_display(self):
         user = self.create_user()
         response = self.client.get(f'/{user}/')

         self.assertEqual(response.status_code, 200)
         self.assertIsInstance(response.context['form'], ClientLoginForm)


    def test_incorrect_data_errors_display(self):

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

    def test_form_display(self):
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/panel/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], ClientChooseVisitForm)