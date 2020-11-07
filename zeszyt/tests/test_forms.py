from django.contrib.auth import authenticate
from django.contrib.auth.models import User
from django.test import TestCase
from ..forms import LoginForm

"""class LoginFormTest(TestCase):
    def test_correct_login_auhorize(self):
        self.user = User.objects.create_user(username='user', password='pass')
        form = LoginForm(data={'username':'user', 'password':'pass'})
        self.assertIn('_auth_user_id', self.client.session)
"""