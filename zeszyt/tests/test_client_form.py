from django.test import TestCase
from unittest import skip
from ..forms import ClientLoginForm
from .base import BaseTest

class LoginTests(BaseTest):


   def test_form_display(self):
        user = self.create_user()
        response = self.client.get(f'/{user}/')

        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], ClientLoginForm)