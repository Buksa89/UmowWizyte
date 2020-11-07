from django.contrib.auth.models import User
from django.test import TestCase

class UserModelTest(TestCase):
    def test_user_saving(self):
        self.user = User.objects.create_user(username='user', password='pass')
        self.assertEqual(self.user, User.objects.first())