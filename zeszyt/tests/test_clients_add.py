from django.contrib.auth.models import User
from django.core.exceptions import ValidationError
from django.test import TestCase
from unittest import skip
from ..forms import AddClientForm
from ..models import Client
from .base import BaseTest

class PanelClientsAddTemplateTests(BaseTest):

    def test_user_add_client_template(self):
        self.authorize_user()
        response = self.client.get('/klienci/nowy/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'panel/add_client.html')

    def test_user_add_client_template_POST(self):
        self.authorize_user()
        response = self.client.post('/klienci/nowy/', data={})
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'panel/add_client.html')


class PanelClientsAddModelTests(TestCase):
    def test_client_saving(self):
        user = User.objects.create()
        client = Client.objects.create(phone_number='111', user=user)
        self.assertEqual(client, Client.objects.first())

    def test_cannot_save_empty_phone_number(self):
        user = User.objects.create()
        with self.assertRaises(ValidationError):
            client = Client(user=user)
            client.full_clean()

    def test_cannot_save_empty_user(self):
        with self.assertRaises(ValidationError):
            client = Client(phone_number='111')
            client.full_clean()

    def test_cannot_save_wrong_number(self):
        with self.assertRaises(ValidationError):
            client = Client(phone_number='aaa')
            client.full_clean()

    def test_client_is_related_to_user(self):
        user = User.objects.create()
        client = Client(phone_number='1111111')
        client.user = user
        client.save()
        self.assertIn(client, user.client_set.all())

    def test_duplicate_phone_numbers_are_invalid(self):
        user = User.objects.create()
        Client.objects.create(user=user, phone_number='111')
        with self.assertRaises(ValidationError):
            client = Client(user=user, phone_number='111')
            client.full_clean()

    def test_CAN_save_same_client_to_different_users(self):
        """Różni użytkownicy powinni mieć możliwość dodania użytkownika o tym samym numerze telefonu"""
        user1 = User.objects.create()
        user2 = User.objects.create(username='2')
        Client.objects.create(user=user1, phone_number='111')
        client = Client(user=user2, phone_number='111', name='name', pin='1111')
        client.full_clean()  # Nie powinien być zgłoszony


class PanelClientsAddFormTests(TestCase):
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