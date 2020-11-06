from django.test import TestCase

class WelcomeScreenTest(TestCase):

    def test_uses_welcome_template(self):
        response = self.client.get('/')
        self.assertTemplateUsed(response, 'welcome.html')

class LoginScreenTest(TestCase):

    def test_uses_login_template(self):
        response = self.client.get('/login/')
        self.assertTemplateUsed(response, 'login.html')

