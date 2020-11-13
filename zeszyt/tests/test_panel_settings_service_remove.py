from datetime import timedelta
from django.contrib.auth.models import User
from django.test import TestCase
from unittest import skip
from .base import BaseTest
from ..models import Service


class PanelSettingsServiceRemoveModelTests(BaseTest):
    def test_client_remove(self):
        user = User.objects.create()
        service = Service.objects.create(duration=timedelta(hours=1), name='usluga', user=user)
        service.delete()
        self.assertFalse(Service.objects.first())

    def test_get_remove_url(self):
        user = User.objects.create()
        service = Service.objects.create(duration=timedelta(hours=1), name='usluga', user=user)
        self.assertEqual(service.get_remove_url(), f'/panel/usun_usluge/{service.id}/')

class PanelSettingsServiceRemoveViewTests(BaseTest):
    def test_user_remove_service(self):
        self.authorize_user('user', 'pass')
        service = Service.objects.create(duration=timedelta(hours=1), name='usluga', user=self.user)
        self.client.get(service.get_remove_url())
        self.assertFalse(Service.objects.first())

    def test_user_cannot_remove_others_services(self):
        """ Jeden użytkownik nie powinien mieć możliwości usunięcia usługi innego, poprzez ręczne wpisanie
        linka z id usługi """
        self.authorize_user('user', 'pass')
        service = Service.objects.create(duration=timedelta(hours=1), name='usluga', user=self.user)
        self.authorize_user('user2', 'pass2')
        self.client.get(service.get_remove_url())
        self.assertTrue(Service.objects.first())