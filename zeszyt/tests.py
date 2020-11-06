from django.contrib.auth.models import User
from django.test import TestCase

class WelcomeScreenTest(TestCase):

    def test_uses_welcome_template(self):
        response = self.client.get('/')
        self.assertTemplateUsed(response, 'welcome.html')

class LoginScreenTest(TestCase):

    def test_uses_login_template(self):
        response = self.client.get('/login/')
        self.assertTemplateUsed(response, 'login.html')

class RedirectsLoginTests(TestCase):

    def test_panel_redirect_to_login_when_user_not_authorized(self):
        response = self.client.get('/panel/')
        self.assertRedirects(response, f'/login/?next=/panel/')

    def test_redirect_after_logout(self):
        self.user = User.objects.create_user(username='testuser', password='12345')
        login = self.client.login(username='testuser', password='12345')
        response = self.client.get('/logout/')
        self.assertRedirects(response, f'/login/')

    def test_redirect_login_to_panel_when_logged(self):
        self.user = User.objects.create_user(username='testuser', password='12345')
        login = self.client.login(username='testuser', password='12345')
        response = self.client.get('/login/')
        self.assertRedirects(response, f'/panel/')


