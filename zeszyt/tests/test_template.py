from django.test import TestCase
from unittest import skip
from .base import BaseTest
from ..models import Client, Service


class DashboardTests(BaseTest):

    def test_user_panel_template(self):
        self.authorize_user()
        response = self.client.get('/panel/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'dashboard/dashboard.html')


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
        self.assertTemplateUsed(response, 'dashboard/clients_add.html')

    def test_user_add_client_template_POST(self):
        self.authorize_user()
        response = self.client.post('/klienci/nowy/', data={})

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'dashboard/clients_add.html')


class DashboardScheduleTests(BaseTest):

    def test_calendar_template(self):
        self.authorize_user()
        response = self.client.get('/terminarz/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'dashboard/schedule_calendar.html')

    def test_calendar_date_url_template(self):
        self.authorize_user()
        response = self.client.get('/terminarz/2020/8')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'dashboard/schedule_calendar.html')



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