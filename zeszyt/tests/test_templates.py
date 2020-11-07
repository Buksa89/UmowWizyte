from django.test import TestCase
from django.contrib.auth.models import User
from unittest import skip
from .base import BaseTest

class Screen(BaseTest):

    def test_welcome_template(self):
        response = self.client.get('/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'welcome.html')

    def test_user_login_template(self):
        response = self.client.get('/login/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'login.html')

    def test_user_panel_template(self):
        self.authorize_user()
        response = self.client.get('/panel/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'panel/panel.html')

    def test_user_clients_template(self):
        self.authorize_user()
        response = self.client.get('/klienci/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'panel/clients.html')

    def test_user_add_client_template(self):
        self.authorize_user()
        response = self.client.get('/klienci/nowy')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'panel/add_client.html')