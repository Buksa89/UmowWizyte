from django.test import TestCase
from unittest import skip
from ..forms import AddClientForm
from .base import BaseTest

class TestTemplate(BaseTest):

    def test_user_add_client_template(self):
        self.authorize_user()
        response = self.client.get('/klienci/nowy')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'panel/add_client.html')

    def test_user_add_client_template_POST(self):
        self.authorize_user()
        response = self.client.post('/klienci/nowy', data={})
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'panel/add_client.html')

class TestForm(TestCase):
    def test_add_client_form_validation_for_blank_fields(self):
        form = AddClientForm(data={'pin':'1111'})
        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['phone_number'], ['Pole nie może być puste'])
        self.assertEqual(form.errors['name'], ['Pole nie może być puste'])

    def test_add_client_form_validation_for_wrong_phone_number(self):
        form = AddClientForm(data={'pin':'1111', 'name':'aaa', 'phone_number':'letters'})
        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['phone_number'], ['Podaj prawidłowy numer telefonu'])

    @skip
    def test_add_client_form_validation_for_duplicate(self):
        pass
        # TODO Formularz powinien walidować duplikaty. Teraz robi to widok
        #AddClientForm(data={'pin': '1111', 'name': 'aaa', 'phone_number': '111'})
        #form = AddClientForm(data={'pin': '1111', 'name': 'aaa', 'phone_number': '111'})
        #self.assertFalse(form.is_valid())
        #print(form.errors)
        #self.assertEqual(form.errors['phone_number'], ['Podaj prawidłowy numer telefonu'])




        # TODO zbyt długie wpisy - overflow erro