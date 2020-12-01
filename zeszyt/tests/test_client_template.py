from django.test import TestCase
from unittest import skip
from .base import BaseTest


class LoginTests(BaseTest):
    def test_login_template_display(self):
        user = self.create_user()
        response = self.client.get(f'/{user}/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_app/login.html')

    def test_login_template_POST(self):
        user = self.create_user('active_1')
        response = self.client.post(f'/{user}/', data={})

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_app/login.html')

    @skip
    def test_login_template_not_active(self):
        # TODO: Strona logowania powinna być nieaktywna dla zablokowanych uzytkownikow
        user = self.create_user()
        response = self.client.get(f'/{user}/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_app/login_not_active.html')