from django.contrib.auth import authenticate
from django.contrib.auth.models import User
from django.test import TestCase
from ..forms import LoginForm
from ..models import Client


class FormTests(TestCase):
    pass
    # TODO: Test czy formularz się wyświetla

class ViewTests(TestCase):
    def test_is_login_correct(self):
        #response = self.client.get('/login/')
        #self.assertEqual(response.status_code, 200)
        #self.assertIsInstance(response.context['form'], LoginForm)
        pass

    def test_incorrect_login_display_errors(self):
        pass
        #self.user = User.objects.create_user(username='user', password='pass')
        #self.user = User.objects.create_user(username='user2', password='pass2', is_active=False)

        #data_results = [{'data': {'username': 'user', 'password': 'wrong_pass'}, 'message': 'Błędny login lub hasło'},
        #                {'data': {'username': 'wrong_username', 'password': 'pass'},
        #                 'message': 'Błędny login lub hasło'},
        #                {'data': {'username': 'user2', 'password': 'pass2'}, 'message': 'Konto zablokowane'},
        #                {'data': {'username': '', 'password': 'pass2'}, 'message': 'Podaj login'},
        #                {'data': {'username': 'user2', 'password': ''}, 'message': 'Podaj hasło'}]
        #for data_result in data_results:
        #    response = self.client.post('/login/', data=data_result['data'])
        #    self.assertContains(response, data_result['message'])

    def test_incorrect_login_auhorize(self):
        pass
        #self.user = User.objects.create_user(username='user', password='pass')

        #response = self.client.post('/login/', data={'username': 'user', 'password': 'wrong_pass'})
        #response2 = self.client.post('/login/', data={'username': 'wrong_username', 'password': 'pass'})
        #self.assertNotIn('_auth_user_id', self.client.session)

    def test_not_active_login_auhorize(self):
        pass
        #self.user = User.objects.create_user(username='user', password='pass', is_active=False)

        #response = self.client.post('/login/', data={'username': 'user', 'password': 'pass'})
        #self.assertNotIn('_auth_user_id', self.client.session)

    def test_correct_login_auhorize(self):
        #self.user = User.objects.create_user(username='user', password='pass')
        #self.client.post('/login/', data={'username': 'user', 'password': 'pass'})
        #self.assertIn('_auth_user_id', self.client.session)
        #self.assertEqual(int(self.client.session['_auth_user_id']), self.user.pk)
        pass

class TemplateTests(TestCase):
    def test_user_login_template(self):
        #response = self.client.get('/login/')
        #self.assertEqual(response.status_code, 200)
        #self.assertTemplateUsed(response, 'login.html')
        pass

    def test_user_login_template_post(self):
        pass
        #response = self.client.post('/login/', data={})
        #self.assertEqual(response.status_code, 200)
        #self.assertTemplateUsed(response, 'login.html')


class RedirectsLoginTests(TestCase):

    def test_panel_redirect_to_login_when_user_not_authorized(self):
        pass
        #response = self.client.get('/panel/')
        #self.assertRedirects(response, f'/login/?next=/panel/')
        #response = self.client.get('/klienci/')
        #self.assertRedirects(response, f'/login/?next=/klienci/')

    def test_redirect_after_logout(self):
        pass
        #self.user = User.objects.create_user(username='testuser', password='12345')
        #login = self.client.login(username='testuser', password='12345')
        #response = self.client.get('/logout/')
        #self.assertRedirects(response, f'/login/')

    def test_redirect_login_to_panel_when_logged(self):
        #self.user = User.objects.create_user(username='testuser', password='12345')
        #login = self.client.login(username='testuser', password='12345')
        #response = self.client.get('/login/')
        #self.assertRedirects(response, f'/panel/')
        pass

