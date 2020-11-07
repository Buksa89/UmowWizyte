from django.contrib.auth import authenticate
from django.contrib.auth.models import User
from django.test import TestCase
from ..forms import LoginForm

class LoginTest(TestCase):
    def test_is_login_correct(self):
        response = self.client.get('/login/')
        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], LoginForm)

    def test_incorrect_login_display_errors(self):
        self.user = User.objects.create_user(username='user', password='pass')
        self.user = User.objects.create_user(username='user2', password='pass2', is_active=False)

        response1 = self.client.post('/login/', data={'username':'user', 'password':'wrong_pass'})
        response2 = self.client.post('/login/', data={'username':'wrong_username', 'password':'pass'})
        response3 = self.client.post('/login/', data={'username':'user2', 'password':'pass2'})
        response4 = self.client.post('/login/', data={'username':'', 'password':'pass2'})
        response5 = self.client.post('/login/', data={'username':'user2', 'password':''})

        self.assertContains(response1, 'Błędny login lub hasło')
        self.assertContains(response2, 'Błędny login lub hasło')
        self.assertContains(response3, 'Konto zablokowane')
        self.assertContains(response4, 'Podaj login')
        self.assertContains(response5, 'Podaj hasło')



    def test_incorrect_login_auhorize(self):
        self.user = User.objects.create_user(username='user', password='pass')

        response = self.client.post('/login/', data={'username':'user', 'password':'wrong_pass'})
        response2 = self.client.post('/login/', data={'username':'wrong_username', 'password':'pass'})
        self.assertNotIn('_auth_user_id', self.client.session)

    def test_not_active_login_auhorize(self):
        self.user = User.objects.create_user(username='user', password='pass', is_active=False)

        response = self.client.post('/login/', data={'username':'user', 'password':'pass'})
        self.assertNotIn('_auth_user_id', self.client.session)

    def test_correct_login_auhorize(self):
        self.user = User.objects.create_user(username='user', password='pass')
        self.client.post('/login/', data={'username':'user', 'password':'pass'})
        self.assertIn('_auth_user_id', self.client.session)
        self.assertEqual(int(self.client.session['_auth_user_id']), self.user.pk)
