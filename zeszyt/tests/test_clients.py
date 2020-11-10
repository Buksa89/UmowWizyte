from django.contrib.auth import authenticate
from django.contrib.auth.models import User
from django.test import TestCase
from unittest import skip
from ..forms import AddClientForm, LoginForm
from ..models import Client
from .base import BaseTest

from django.test import TestCase
from django.contrib.auth.models import User
from unittest import skip
from .base import BaseTest


class Screen(BaseTest):

    def test_user_clients_template(self):
        self.authorize_user()
        response = self.client.get('/klienci/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'panel/clients.html')
