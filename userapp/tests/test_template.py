from django.test import TestCase
from unittest import skip
from ..forms import LoginForm
from .base import BaseTest


class LoginTests(TestCase):

    def test_login_template_display(self):
        response = self.client.get('/login/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'login.html')

    def test_login_template_POST(self):
        response = self.client.post('/login/', data={})

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'login.html')


class DashboardTests(BaseTest):

    def test_dashboard_template_display(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.get('/panel/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'dashboard.html')


class DashboardClientsTests(BaseTest):

    def test_dashboard_clients_template_display(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.get('/klienci/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'clients.html')


    """ Add client """

    def test_dashboard_clients_add_template_display(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.get('/klienci/nowy/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'clients_add.html')

    def test_dashboard_clients_add_template_POST(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.post('/klienci/nowy/', data={})

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'clients_add.html')


class DashboardSettingsTests(BaseTest):

    def test_dashboard_settings_template(self):
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.get('/ustawienia/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'settings.html')

    def test_dashboard_settings_template_POST(self):
        # TODO: Tu są trzy formularze. Dla każdego trzeba sprawdzić templatkę
        user = self.create_user()
        self.authorize_user(user)
        response = self.client.post('/ustawienia/', data={})

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'settings.html')
