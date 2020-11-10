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

class ViewTests(BaseTest):

    def test_user_cannot_remove_others_clients(self):
        self.authorize_user('user', 'pass')
        client_ = Client.objects.create(phone_number='111', user=self.user)
        self.authorize_user('user2', 'pass2')
        response = self.client.get(client_.get_remove_url())
        self.assertTrue(Client.objects.first())