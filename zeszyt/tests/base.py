from django.contrib.auth.models import User
from django.test import TestCase
from django.test import Client
from ..models import Client as ClientModel

class BaseTest(TestCase):
    def authorize_user(self, username='user', password='pass'):
        """ Autoryzacja użytkownika do testów
        Jeśli użytkownik o padanej nazwie nie istnieje, zostaje utworzony"""
        if not User.objects.filter(username=username):
            self.user = User.objects.create_user(username=username)
        self.client.force_login(self.user)

    def authorize_client(self, phone=None, user=None):
        if not phone and not user:
            self.user = User.objects.create_user(username='user', password='pass')
            client = ClientModel.objects.create(user=self.user, phone_number='1111', pin='1111')
        session = self.client.session
        session['client_authorized'] = {'phone': '1111', 'user':'user'}
        session.save()