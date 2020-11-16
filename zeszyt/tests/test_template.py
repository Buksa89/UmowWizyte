from django.test import TestCase
from unittest import skip
from .base import BaseTest


class DashboardTests(BaseTest):

    def test_user_panel_template(self):
        self.authorize_user()
        response = self.client.get('/panel/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'dashboard/panel.html')


class DashboardClientsTests(BaseTest):

    def test_user_clients_template(self):
        self.authorize_user()
        response = self.client.get('/klienci/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'dashboard/clients.html')


    """ Add client """

    def test_user_add_client_template(self):
        self.authorize_user()
        response = self.client.get('/klienci/nowy/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'dashboard/add_client.html')

    def test_user_add_client_template_POST(self):
        self.authorize_user()
        response = self.client.post('/klienci/nowy/', data={})

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'dashboard/add_client.html')


    """ Remove client """
    #TODO: Dodaj test remove


class DashboardSettingsTests(BaseTest):

    def test_panel_settings_template(self):
        self.authorize_user()
        response = self.client.get('/ustawienia/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'dashboard/settings.html')

    def test_panel_settings_template_POST(self):
        self.authorize_user()
        response = self.client.post('/ustawienia/', data={})

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'dashboard/settings.html')


class UserLoginTests(TestCase):

    def test_user_login_template(self):
        response = self.client.get('/login/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'login.html')

    def test_user_login_template_post(self):
        response = self.client.post('/login/', data={})

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'login.html')