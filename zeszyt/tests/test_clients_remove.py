from django.test import TestCase
from django.contrib.auth.models import User
from unittest import skip
from .base import BaseTest
from ..models import Client


class ModelTests(BaseTest):
    def test_client_remove(self):
        user = User.objects.create()
        client = Client.objects.create(phone_number='111', user=user)
        client.delete()
        self.assertFalse(Client.objects.first())

    def test_get_remove_url(self):
        user = User.objects.create()
        client = Client.objects.create(phone_number='111', user=user)
        self.assertEqual(client.get_remove_url(), f'/klienci/usun/{client.id}')

class ViewTests(BaseTest):

    def test_user_remove_client(self):
        self.authorize_user('user', 'pass')
        client_ = Client.objects.create(phone_number='111', user=self.user)
        response = self.client.get(client_.get_remove_url())
        self.assertFalse(Client.objects.first())

    def test_user_cannot_remove_others_clients(self):
        """Użytkownik nie powinien mieć możliwości usunięcia klientów innego użytkownika
        poprzez podanie jego id w linku"""
        self.authorize_user('user', 'pass')
        client_ = Client.objects.create(phone_number='111', user=self.user)
        self.authorize_user('user2', 'pass2')
        response = self.client.get(client_.get_remove_url())
        self.assertTrue(Client.objects.first())