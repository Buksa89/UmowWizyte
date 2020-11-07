from django.contrib.auth.models import User
from django.test import TestCase

class Screen(TestCase):

    def test_uses_welcome_template(self):
        response = self.client.get('/')
        self.assertTemplateUsed(response, 'welcome.html')

    def test_uses_login_template(self):
        response = self.client.get('/login/')
        self.assertTemplateUsed(response, 'login.html')