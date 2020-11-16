from datetime import timedelta
from django.contrib.auth.models import User
from django.core.exceptions import ValidationError
from django.db.utils import IntegrityError
from django.test import TestCase
from unittest import skip
from ..models import Client, Service
from .base import BaseTest


class ClientsTests(BaseTest):

    """ Add clients """

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
        user = User.objects.create()

        with self.assertRaises(ValidationError):
            client = Client(phone_number='aaa', user=user)
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
        # TODO: Zorientowac się, jaki test tu zastosować
        user1 = User.objects.create()
        user2 = User.objects.create(username='2')
        Client.objects.create(user=user1, phone_number='111')
        client = Client(user=user2, phone_number='111', name='name', pin='1111')
        client.full_clean()  # Nie powinien być zgłoszony


    """ Remove clients """

    def test_client_remove(self):
        user = User.objects.create()
        client = Client.objects.create(phone_number='111', user=user)
        client.delete()

        self.assertFalse(Client.objects.first())

    def test_get_remove_url(self):
        user = User.objects.create()
        client = Client.objects.create(phone_number='111', user=user)

        self.assertEqual(client.get_remove_url(), f'/klienci/usun/{client.id}/')


class ServiceTests(BaseTest):

    """ Add service tests """
    def test_service_saving(self):
        user = User.objects.create()
        service = Service.objects.create(duration=timedelta(hours=1), name='usługa', user=user)

        self.assertEqual(service, Service.objects.first())

    def test_cannot_save_empty_user(self):
        with self.assertRaises(ValidationError):
            service = Service(duration=timedelta(hours=1), name='usługa')
            service.full_clean()

    def test_cannot_save_empty_name(self):
        user = User.objects.create()

        with self.assertRaises(ValidationError):
            service = Service(duration=timedelta(hours=1), user=user)
            service.full_clean()

    def test_cannot_save_empty_duration(self):
        user = User.objects.create()

        with self.assertRaises(AttributeError):
            Service.objects.create(name='usługa', user=user)

    def test_duplicate_service_name(self):
        user = User.objects.create()
        Service.objects.create(user=user, duration=timedelta(hours=1), name='usługa')

        with self.assertRaises(ValidationError):
            service = Service(user=user, duration=timedelta(hours=1), name='usługa')
            service.full_clean()

    def test_long_service_name(self):
        name = "Muvaffakiyetsizleştiricilestiriveremeyebileceklerimizdenmissinizcesine"
        user = User.objects.create()

        with self.assertRaises(ValidationError):
            service = Service(user=user, duration=timedelta(hours=1), name=name)
            service.full_clean()

    def test_cannot_save_wrong_duration(self):
        user = User.objects.create()

        with self.assertRaises(ValidationError):
            service = Service(user=user, duration='ffff', name='usługa')
            service.full_clean()

    def test_service_is_related_to_user(self):
        user = User.objects.create()
        service = Service(duration=timedelta(hours=1), name='usługa')
        service.user = user
        service.save()

        self.assertIn(service, user.service_set.all())

    def test_CAN_save_same_service_to_different_users(self):
        # TODO: Uspuełnij o prawidłowy test
        user1 = User.objects.create()
        user2 = User.objects.create(username='2')
        Service.objects.create(user=user1, duration=timedelta(hours=1), name='usługa')
        service = Service(user=user2, duration=timedelta(hours=1), name='usługa')
        service.full_clean()  # Nie powinien być zgłoszony

    """ Remove service tests """

    def test_client_remove(self):
        user = User.objects.create()
        service = Service.objects.create(duration=timedelta(hours=1), name='usluga', user=user)
        service.delete()

        self.assertFalse(Service.objects.first())

    def test_get_remove_url(self):
        user = User.objects.create()
        service = Service.objects.create(duration=timedelta(hours=1), name='usluga', user=user)

        self.assertEqual(service.get_remove_url(), f'/ustawienia/usun_usluge/{service.id}/')


class UserTest(TestCase):

    def test_user_saving(self):
        user = User.objects.create_user(username='user', password='pass')

        self.assertEqual(user, User.objects.first())
        # TODO: Test zmiany hasła użytkownika