from django.test import TestCase
from unittest import skip
from ..forms import AddClientForm, AddServiceForm, LoginForm, RegistrationForm
from .base import BaseTest
import userapp.language as l

class RegistrationTests(BaseTest):
    def test_form_display(self):
        response = self.client.get('/registration/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], RegistrationForm)

    def test_form_validation_blank(self):
        form = RegistrationForm(data={})

        self.assertFalse(form.is_valid())
        self.assertIn(form.errors['username'][0], l.FIELD_REQUIRED)
        self.assertIn(form.errors['email'][0], l.FIELD_REQUIRED)
        self.assertIn(form.errors['password'][0], l.FIELD_REQUIRED)
        self.assertIn(form.errors['password2'][0], l.FIELD_REQUIRED)

    def test_form_validation_too_long(self):
        data = {'username': self.LONG_STRING(41),
                'first_name': self.LONG_STRING(41),
                'last_name': self.LONG_STRING(41),
                'email': self.LONG_EMAIL,
                'password': self.LONG_STRING(41),
                'password2': self.LONG_STRING(41)}

        form = RegistrationForm(data=data)

        self.assertFalse(form.is_valid())
        self.assertIn(form.errors['username'][0], l.FIELD_TOO_LONG)
        self.assertIn(form.errors['first_name'][0], l.FIELD_TOO_LONG)
        self.assertIn(form.errors['last_name'][0], l.FIELD_TOO_LONG)
        self.assertIn(form.errors['email'][0], l.FIELD_TOO_LONG)
        self.assertIn(form.errors['password'][0], l.FIELD_TOO_LONG)
        self.assertIn(form.errors['password2'][0], l.FIELD_TOO_LONG)

    def test_form_validation_exists(self):
        data = {'username': self.LONG_STRING(41),
                'email': self.LONG_EMAIL,
                'password': self.CORRECT_PASSWORD,
                'password2': self.CORRECT_PASSWORD}

        form = RegistrationForm(data=data)

        self.assertFalse(form.is_valid())
        self.assertIn(form.errors['username'][0], l.FIELD_TOO_LONG)
        self.assertIn(form.errors['first_name'][0], l.FIELD_TOO_LONG)

        pass

    def test_form_validation_password_strong(self):
        pass

    def test_form_validation_email_correct(self):
        pass

    def test_form_validation_password_match(self):
        pass



'''class DashboardClientsTests(BaseTest):

    """ Add Clients """

    def test_add_client_form_display(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.get('/klienci/nowy/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], AddClientForm)

    def test_add_client_errors_for_blank_fields(self):
        client = self.clients_full_data['active_1']
        form = AddClientForm(data={'pin':client['pin']})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['phone_number'], ['Pole nie może być puste'])
        self.assertEqual(form.errors['name'], ['Pole nie może być puste'])

    def test_add_client_errors_for_long_fields(self):
        client = self.clients_full_data['active_1']
        form = AddClientForm(data={'pin':client['pin'],
                                   'name':self.LONG_STRING(21),
                                   'phone_number':self.LONG_DIGIT(21),
                                   'email':self.LONG_EMAIL,
                                   'surname':self.LONG_STRING(41),
                                   'description':self.LONG_STRING(201)})
        result = {'phone_number': 'Numer jest za długi',
                  'name':'Nazwa jest za długa',
                  'email':'Email jest za długi',
                  'surname':'Nazwisko jest za długie',
                  'description':'Opis jest za długi',
                  }
        self.assertFalse(form.is_valid())
        for field, message in result.items():
            self.assertEqual(form.errors[field], [message])


    def test_add_client_errors_for_incorrect_fields(self):
        client = self.clients['active_1']
        client_full = self.clients_full_data['active_1']
        form = AddClientForm(data={'pin':client_full['pin'], 'name':client['name'],
                                   'phone_number':self.WRONG_DATA, 'email':self.WRONG_DATA})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['phone_number'], ['Podaj prawidłowy numer telefonu'])
        self.assertEqual(form.errors['email'], ['Email nieprawidłowy'])


    @skip #TODO Naprawić ten test jak juz ogarne formularz
    def test_add_client_error_for_duplicate(self):
        user = self.create_user()
        self.create_client(user, 'active_1')
        form = AddClientForm(data=self.clients['active_1'])
        self.assertFalse(form.is_valid())
        self.assertEqual(form.non_field_errors(), ['Posiadasz już klienta z tym numerem telefonu'])


class DashboardSettingsTests(BaseTest):

    """ Service """

    def test_service_uses_service_form(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.get('/ustawienia/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['service_form'], AddServiceForm)


    def test_add_service_form_validation_for_blank_fields(self):
        user = self.create_user()
        self.authorize_user(user)
        form = AddServiceForm(data={})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['duration'], ['Pole nie może być puste'])
        self.assertEqual(form.errors['name'], ['Pole nie może być puste'])


    def test_add_service_form_validation_for_wrong_duration(self):
        user = self.create_user()
        self.authorize_user(user)
        service = self.services['short_1']
        form = AddServiceForm(data={'duration':self.WRONG_DATA,
                                    'name':service['name']})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['duration'], ['Nie kombinuj!'])


    def test_add_service_form_validation_for_long_names(self):
        user = self.create_user()
        self.authorize_user(user)
        service = self.services['short_1']
        form = AddServiceForm(data={'duration':self.WRONG_DATA,
                                    'name':self.LONG_STRING(61)})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['name'], ['Nazwa jest za długa'])

        # TODO Formularz powinien walidować duplikaty. Teraz robi to widok
        #  def test_add_service_form_validation_for_duplicate(self):


    """ WorkTime """

    def test_service_uses_work_time_form(self):
        user = self.create_user()
        self.authorize_user(user)
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

#TODO        self.assertIn('Popraw godziny pracy', form.errors['__all__'][0])
        self.assertIn('Popraw możliwość wyboru terminów', form.errors['__all__'][0])


    def test_work_time_form_validation_for_negative_days(self):
        form = WorkTimeForm(data={'start_time':'01:00', 'end_time':'01:15', 'earliest_visit': -14, 'latest_visit':-1})

        self.assertFalse(form.is_valid())
        #TODO: END Spolszczenie tego błędu
        self.assertEqual(form.errors['earliest_visit'], ['Ensure this value is greater than or equal to 0.'])
        self.assertEqual(form.errors['latest_visit'], ['Ensure this value is greater than or equal to 0.'])


class LoginTests(BaseTest):
    def test_form_display(self):
        response = self.client.get('/login/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], LoginForm)


    def test_incorrect_data_errors_display(self):
        user = self.users['active_1']
        data_results = [{'data': {'username': self.EMPTY, 'password': user['password']}, 'field':'username', 'message': 'Podaj login'},
                        {'data': {'username': user['username'], 'pasword': self.EMPTY}, 'field':'password', 'message': 'Podaj hasło'},
                        ]

        for data_result in data_results:
            form = LoginForm(data=data_result['data'])
            self.assertFalse(form.is_valid())
            self.assertEqual(form.errors[data_result['field']], [data_result['message']])
'''