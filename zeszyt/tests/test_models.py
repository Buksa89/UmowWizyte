from django.contrib.auth.models import User
from django.core.exceptions import ValidationError
from django.db.utils import IntegrityError
from django.test import TestCase
from ..models import Client


class UserModelTest(TestCase):
    def test_user_saving(self):
        user = User.objects.create_user(username='user', password='pass')
        self.assertEqual(user, User.objects.first())
        # TODO: Test zmiany hasła użytkownika

class ClientModelTest(TestCase):
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
        user1 = User.objects.create()
        user2 = User.objects.create(username='2')
        client = Client.objects.create(user=user1, phone_number='111')
        client = Client(user=user2, phone_number='111', name='name', pin='1111')
        client.full_clean()  # Nie powinien być zgłoszony

    def test_get_remove_url(self):
        user = User.objects.create()
        client = Client.objects.create(phone_number='111', user=user)
        self.assertEqual(client.get_remove_url(), f'/klienci/usun/{client.id}')

    def test_remove_client(self):
        user = User.objects.create()
        client = Client.objects.create(phone_number='111', user=user)

    def test_client_edit(self):
        pass
        #TODO: Test edycji klienta