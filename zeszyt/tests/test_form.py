from django.test import TestCase
from ..forms import AddClientForm, AddServiceForm, LoginForm
from .base import BaseTest

class DashboardClientsTests(BaseTest):

    """ Add Clients """

    def test_clients_uses_item_form(self):
        self.authorize_user()
        response = self.client.get('/klienci/nowy/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], AddClientForm)

    def test_add_client_form_validation_for_blank_fields(self):
        form = AddClientForm(data={'pin':'1111'})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['phone_number'], ['Pole nie może być puste'])
        self.assertEqual(form.errors['name'], ['Pole nie może być puste'])

    def test_add_client_form_validation_for_long_fields(self):
        form = AddClientForm(data={'pin':'1111',
                                   'name':'a'*100,
                                   'phone_number':'1'*100,
                                   'email':'a'*100,
                                   'surname':'a'*100,
                                   'description':'a'*201})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['phone_number'], ['Numer jest za długi'])
        self.assertEqual(form.errors['name'], ['Nazwa jest za długa'])
        self.assertEqual(form.errors['email'], ['Email nieprawidłowy', 'Email jest za długi'])
        self.assertEqual(form.errors['surname'], ['Nazwisko jest za długie'])
        self.assertEqual(form.errors['description'], ['Opis jest za długi'])

    def test_add_client_form_validation_for_wrong_phone_number(self):
        form = AddClientForm(data={'pin':'1111', 'name':'aaa', 'phone_number':'letters'})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['phone_number'], ['Podaj prawidłowy numer telefonu'])

    # TODO Formularz powinien walidować duplikaty. Teraz robi to widok
    #  def test_add_service_form_validation_for_duplicate(self):
        #AddClientForm(data={'pin': '1111', 'name': 'aaa', 'phone_number': '111'})
        #form = AddClientForm(data={'pin': '1111', 'name': 'aaa', 'phone_number': '111'})
        #self.assertFalse(form.is_valid())
        #print(form.errors)
        #self.assertEqual(form.errors['phone_number'], ['Podaj prawidłowy numer telefonu'])


class DashboardSettingsTests(BaseTest):

    def test_service_uses_item_form(self):
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
        form = AddServiceForm(data={'duration':'ffff', 'name':'usługa'})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['duration'], ['Nie kombinuj!'])

    def test_add_service_form_validation_for_long_names(self):
        name = "Muvaffakiyetsizleştiricilestiriveremeyebileceklerimizdenmissinizcesine"
        form = AddServiceForm(data={'duration':'12', 'name':name})

        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['name'], ['Nazwa jest za długa'])

        # TODO Formularz powinien walidować duplikaty. Teraz robi to widok
        #  def test_add_service_form_validation_for_duplicate(self):


class UserLoginTests(TestCase):
    def test_is_login_correct(self):
        response = self.client.get('/login/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], LoginForm)