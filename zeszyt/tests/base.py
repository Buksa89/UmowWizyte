from django.contrib.auth.models import User
from django.test import TestCase
from django.test import Client

class BaseTest(TestCase):
    def authorize_user(self, username='user', password='pass'):
        if not User.objects.filter(username=username):
            self.user = User.objects.create_user(username=username)
        self.user.set_password(password)
        self.user.save()
        self.client.login(username=username, password=password)