from django.test import TestCase
from unittest import skip
from userapp.tests.base import BaseTest


class LoginTests(BaseTest):
    def test_login_template_display(self):
        user = self.create_user()
        response = self.client.get(f'/{user}/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_login.html')

    def test_login_template_POST(self):
        user = self.create_user('active_1')
        response = self.client.post(f'/{user}/', data={})

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_login.html')

    def test_login_template_not_active(self):
        """ If user is not active, his app shouldnt be available for clients """
        user = self.create_user('not_active')
        response = self.client.get(f'/{user}/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_login_not_active.html')

class DashboardTests(BaseTest):
    def test_dashboard_template_display(self):
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/panel/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_dashboard.html')
