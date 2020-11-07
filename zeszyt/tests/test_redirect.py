from django.contrib.auth.models import User
from django.test import TestCase


class RedirectsLoginTests(TestCase):

    def test_panel_redirect_to_login_when_user_not_authorized(self):
        response = self.client.get('/panel/')
        self.assertRedirects(response, f'/login/?next=/panel/')
        response = self.client.get('/klienci/')
        self.assertRedirects(response, f'/login/?next=/klienci/')

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

class RejectUserRequest(TestCase):
    def test_404(self):
        # TODO: Dowiedzieć się, dlaczego wyrzuca 404, zamiast robić redirect na login_screen
        response = self.client.get('/klienci/nowy/')
        self.assertEqual(response.status_code, 404)