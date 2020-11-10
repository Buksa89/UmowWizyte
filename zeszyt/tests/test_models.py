from django.contrib.auth.models import User
from django.core.exceptions import ValidationError
from django.db.utils import IntegrityError
from django.test import TestCase
from ..models import Client


class UserModelTest(TestCase):
    def test_user_saving(self):
        user = User.objects.create_user(username='user', password='pass')
        self.assertEqual(user, User.objects.first())
        # TODO: Test zmiany hasła użytkownika





    def test_client_edit(self):
        pass
        #TODO: Test edycji klienta