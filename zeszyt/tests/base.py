from datetime import timedelta
from django.contrib.auth.models import User
from django.test import TestCase
from django.test import Client
from ..models import Client as ClientModel
from ..models import Service




class BaseTest(TestCase):
    users = {'active_1': {'username':'Rzesiscie1','password': 'RzesPass12', 'is_active':True},
             'active_2': {'username':'Brodacz','password': 'BarPass12', 'is_active':True},
             'not_active': {'username':'LeniwyLeszek','password': 'Nieaktywny', 'is_active':False},
    }
    clients = {'active_1': {'phone_number':'111111', 'email':'hania@rz1.pl', 'name':'HannaR', 'surname':'Gronkiewicz-Walc', 'description':'Pierwsza klientka'},
               'active_2': {'phone_number':'111222', 'email':'madzia@rz2.pl', 'name':'Magdalena', 'surname':'Ogórek', 'description':'Druga klientka'},
               'active_3': {'phone_number':'111333', 'email':'janusz@bar1.pl', 'name':'Janusz', 'surname':'Korwin-Mikke', 'description':'Trzeci klient'},
               'active_4': {'phone_number':'111444', 'email':'stas@bar2.pl', 'name':'Stanisław', 'surname':'Żółtek', 'description':'Czwarty klient'},
               'not_active': {'phone_number':'222111', 'email':'blok@bar1.pl', 'name':'Blok1', 'surname':'Blok1', 'description':'Zablokowana klientka'},
    }
    clients_full_data = {'active_1': {'pin':'1111', 'is_active':True},
                         'active_2': {'pin':'1122', 'is_active':True},
                         'active_3': {'pin':'1133', 'is_active':True},
                         'active_4': {'pin':'1144', 'is_active':True},
                         'not_active': {'pin':'2211', 'is_active':False},
                         }
    services = {'short_1': {'name': 'Henna', 'duration': timedelta(hours=1), 'is_active': True,},
                'short_2': {'name': 'Paznokcie', 'duration': timedelta(hours=1), 'is_active': True,},
                'long_1': {'name': 'Rzęsy', 'duration': timedelta(hours=8), 'is_active': True,},
                'long_2': {'name': 'Lifting', 'duration': timedelta(hours=8), 'is_active': True,},
                'not_active': {'name': 'Pogaduchy', 'duration': timedelta(hours=1), 'is_active': False,},

    }

    ANY_PHONE = 999999999
    ANY_PIN = 9999
    EMPTY = ''
    WRONG_DATA = 'wrong_data'
    def LONG_DIGIT(self, n): return '1'*n
    def LONG_STRING(self, n): return 'a'*n
    LONG_EMAIL = 'a' * 31 + '@gmail.com'
    DURATION_MIN = timedelta(hours=0)
    DURATION_MAX = timedelta(hours=9)


    def authorize_user(self, user):
        self.client.force_login(user)


    def create_user(self, user='active_1'):
        return User.objects.create_user(username=self.users[user]['username'],
                                        password=self.users[user]['password'],
                                        is_active=self.users[user]['is_active'])

    def create_client(self, user, client='active_1'):
        return ClientModel.objects.create(user=user,
                                          phone_number=self.clients[client]['phone_number'],
                                          email=self.clients[client]['email'],
                                          name=self.clients[client]['name'],
                                          surname=self.clients[client]['surname'],
                                          description=self.clients[client]['description'],
                                          pin=self.clients_full_data[client]['pin'],
                                          is_active=self.clients_full_data[client]['is_active'])

    def authorize_client(self, client):
        session = self.client.session
        session['client_authorized'] = {'phone': client.phone_number, 'user':client.user.username}
        session.save()

    def create_service(self, user, service='short_1'):
        return Service.objects.create(user=user,
                                          name=self.services[service]['name'],
                                          duration=self.services[service]['duration'],
                                          is_active=self.services[service]['is_active'])



