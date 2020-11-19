from django.contrib.auth.models import User
from django.test import TestCase
from unittest import skip
from ..forms import AddClientForm, AddServiceForm, ClientChooseVisitForm, ClientLoginForm, LoginForm, WorkTimeForm
from .base import BaseTest

client_data = {'CORRECT_PIN': '1111',
               'CORRECT_NAME': 'Hania',
               'CORRECT_PHONE_NUMBER': '222222222',
               'WRONG_PHONE_NUMBER': 'wrong_phon123',
               'WRONG_EMAIL': 'wrong_ema',
               'LONG_PHONE_NUMBER': '1' * 21,
               'LONG_NAME': '1' * 21,
               'LONG_EMAIL': 'a' * 31 + '@gmail.com',
               'LONG_SURNAME': 'a' * 41,
               'DESCRIPTION': 'a' * 201,
               }

service_data = {'WRONG_DURATION':'wrong',
                'LONG_NAME':'a'*61,
                'CORRECT_DURATION':'01:00',
                'CORRECT_NAME':'Rzęsy',
}

class ClientLoginTests(BaseTest):

   def test_display_form(self):
        user = self.create_user()
        response = self.client.get(f'/{user}/')
        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], ClientLoginForm)


class ClientDashboardTests(BaseTest):

    def test_display_form(self):
        self.authorize_client()
        response = self.client.get(f'/{self.user}/')
        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], ClientChooseVisitForm)


class DashboardClientsTests(BaseTest):

    """ Add Clients """

    def test_clients_uses_item_form(self):
        self.authorize_user()
        response = self.client.get('/klienci/nowy/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], AddClientForm)

    def test_add_client_form_validation_for_blank_fields(self):
        form = AddClientForm(data={'pin':client_data['CORRECT_PIN']})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['phone_number'], ['Pole nie może być puste'])
        self.assertEqual(form.errors['name'], ['Pole nie może być puste'])

    def test_add_client_form_validation_for_long_fields(self):
        form = AddClientForm(data={'pin':client_data['CORRECT_PIN'],
                                   'name':client_data['LONG_NAME'],
                                   'phone_number':client_data['LONG_PHONE_NUMBER'],
                                   'email':client_data['LONG_EMAIL'],
                                   'surname':client_data['LONG_SURNAME'],
                                   'description':client_data['DESCRIPTION']})
        result = {'phone_number': 'Numer jest za długi',
                  'name':'Nazwa jest za długa',
                  'email':'Email jest za długi',
                  'surname':'Nazwisko jest za długie',
                  'description':'Opis jest za długi',
                  }
        self.assertFalse(form.is_valid())
        for field, message in result.items():
            self.assertEqual(form.errors[field], [message])

    def test_add_client_form_validation_for_wrong_phone_number(self):
        form = AddClientForm(data={'pin':client_data['CORRECT_PIN'], 'name':client_data['CORRECT_NAME'],
                                   'phone_number':client_data['WRONG_PHONE_NUMBER']})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['phone_number'], ['Podaj prawidłowy numer telefonu'])

    def test_add_client_form_validation_for_wrong_email(self): #POPRAW
        form = AddClientForm(data={'phone_number':client_data['CORRECT_PHONE_NUMBER'], 'pin':client_data['CORRECT_PIN'],
                                    'name':client_data['CORRECT_NAME'], 'email':client_data['WRONG_EMAIL']})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['email'], ['Email nieprawidłowy'])

    #TODO Naprawić ten test jak juz ogarne formularz
    @skip
    def test_add_service_form_validation_for_duplicate(self):
        user = self.create_user(user='ok1')
        self.create_client(user, client='cl_ok1')
        form = AddClientForm(data=self.clients['ok1']['cl_ok1'])
        self.assertFalse(form.is_valid())
        self.assertEqual(form.non_field_errors(), ['Posiadasz już klienta z tym numerem telefonu'])


class DashboardSettingsTests(BaseTest):

    def test_service_uses_service_form(self):
        self.authorize_user()
        response = self.client.get('/ustawienia/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['service_form'], AddServiceForm)

    def test_add_service_form_validation_for_blank_fields(self):
        form = AddServiceForm(data={})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['duration'], ['Pole nie może być puste'])
        self.assertEqual(form.errors['name'], ['Pole nie może być puste'])

    def test_add_service_form_validation_for_wrong_duration(self):
        form = AddServiceForm(data={'duration':service_data['WRONG_DURATION'],
                                    'name':service_data['CORRECT_NAME']})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['duration'], ['Nie kombinuj!'])

    def test_add_service_form_validation_for_long_names(self):
        form = AddServiceForm(data={'duration':service_data['CORRECT_DURATION'], 'name':service_data['LONG_NAME']})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['name'], ['Nazwa jest za długa'])

        # TODO Formularz powinien walidować duplikaty. Teraz robi to widok
        #  def test_add_service_form_validation_for_duplicate(self):

    def test_service_uses_work_time_form(self):
        self.authorize_user()
        response = self.client.get('/ustawienia/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['work_time_form'], WorkTimeForm)

    def test_work_time_form_validation_for_blank_fields(self):
        form = WorkTimeForm(data={})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['earliest_visit'], ['Pole nie może być puste'])
        self.assertEqual(form.errors['latest_visit'], ['Pole nie może być puste'])

    def test_work_time_form_validation_for_wrong_fields(self):
        form = WorkTimeForm(data={'start_time':'01:00', 'end_time':'00:15', 'earliest_visit': 14, 'latest_visit':1})

        self.assertIn('Popraw godziny pracy', form.errors['__all__'][0])
        self.assertIn('Popraw możliwość wyboru terminów', form.errors['__all__'][0])

    def test_work_time_form_validation_for_wrong_fields(self):
        form = WorkTimeForm(data={'start_time':'01:00', 'end_time':'01:15', 'earliest_visit': -14, 'latest_visit':-1})

        self.assertFalse(form.is_valid())
        #TODO: END Spolszczenie tego błędu
        self.assertEqual(form.errors['earliest_visit'], ['Ensure this value is greater than or equal to 0.'])
        self.assertEqual(form.errors['latest_visit'], ['Ensure this value is greater than or equal to 0.'])



class UserLoginTests(TestCase):
    def test_is_login_correct(self):
        response = self.client.get('/login/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], LoginForm)