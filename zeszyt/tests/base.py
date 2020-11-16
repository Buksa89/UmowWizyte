from datetime import timedelta
from django.contrib.auth.models import User
from django.test import TestCase
from django.test import Client
from ..models import Client as ClientModel
from ..models import Service



class BaseTest(TestCase):

    def setUp(self):
        self.users = {'ok1': {'username':'Rzesiscie1','password': 'RzesPass12', 'is_active':True},
                      'ok2': {'username':'Barber2','password': 'BarPass12', 'is_active':True},
                      'not_active': {'username':'Nieaktywny','password': 'Nieaktywny', 'is_active':False}}

        self.clients = {'ok1': {
                            'cl_ok1':{
                                'phone_number':'111111',
                                'email':'hania@rz1.pl',
                                'name':'HannaRz1',
                                'surname':'Gronkiewicz-Walc',
                                'description':'Pierwsza klientka firmy Rzęsiście'
                            },
                            'cl_ok2':{
                                'phone_number':'111222',
                                'email':'madzia@rz2.pl',
                                'name':'MagdalenaRz2',
                                'surname':'Ogórek',
                                'description':'Druga klientka firmy Rzęsiście'
                            },
                            'cl_not_active':{
                                'phone_number':'111333',
                                'email':'blok@rz3.pl',
                                'name':'BlokRz',
                                'surname':'',
                                'description':'Zablokowana klientka firmy Rzęsiście'
                            }
                        },
                        'ok2': {
                            'cl_ok1': {
                                'phone_number':'222111',
                                'email':'janusz@bar1.pl',
                                'name':'JanuszB1',
                                'surname':'Korwin-Mikke',
                                'description':'Pierwszy klient firmy Barber'
                            },
                            'cl_ok2': {
                                'phone_number':'222222',
                                'email':'stas@bar2.pl',
                                'name':'StanisławB2',
                                'surname':'Żółtek',
                                'description':'Drugi klient firmy Barber'
                            },
                            'cl_not_active': {
                                'phone_number':'222333',
                                'email':'blok@bar3.pl',
                                'name':'BlokB',
                                'surname':'',
                                'description':'Drugi klient firmy Barber'
                            }
                        }
                        }

        self.clients_full = {'ok1': {
                            'cl_ok1':{
                                'pin':'1111',
                                'is_active':True
                            },
                            'cl_ok2':{
                                'pin':'1122',
                                'is_active':True
                            },
                            'cl_not_active':{
                                'pin':'1133',
                                'is_active':False
                            }
                        },
                             'ok2': {
                            'cl_ok1': {
                                'pin':'2211',
                                'is_active':True
                            },
                            'cl_ok2': {
                                'pin':'2222',
                                'is_active':True
                            },
                            'cl_not_active': {
                                'pin':'2233',
                                'is_active':True
                            }
                        }
                        }

        self.services = {'ok1': {
                            'serv_ok1': {
                                'name': 'Lifting1',
                                'duration': timedelta(hours=1),
                                'is_active': True,
                            },
                            'serv_ok2': {
                                'name': 'Doklejanie2',
                                'duration': timedelta(minutes=150),
                                'is_active': True,
                            },
                            'serv_ok3': {
                                'name': 'Paznokcie3',
                                'duration': timedelta(15),
                                'is_active': True,
                            },
                            'serv_ok4': {
                                'name': 'Pogaduchy4',
                                'duration': timedelta(hours=5),
                                'is_active': True,
                            },
                            'serv_not_active': {
                                'name': 'SpankoNA',
                                'duration': timedelta(hours=1),
                                'is_active': False,
                            }
                        },
                         'ok2': {
                            'serv_ok1': {
                                'name': 'Golenie1',
                                'duration': timedelta(minutes=30),
                                'is_active': True,
                            },
                            'serv_ok2': {
                                'name': 'Wycinka drzewa2',
                                'duration': timedelta(minutes=150),
                                'is_active': True,
                            },
                            'serv_ok3': {
                                'name': 'Herbatka3',
                                'duration': timedelta(minutes=15),
                                'is_active': True,
                            },
                            'serv_ok4': {
                                'name': 'Polowanie na niedźwiedzia4',
                                'duration': timedelta(hours=8),
                                'is_active': True,
                            },
                            'serv_not_active': {
                                'name': 'Ucieczka przed niedźwiedziemNA',
                                'duration': timedelta(hours=3),
                                'is_active': False,
                            }
                        }
                        }

    def authorize_user(self, user='ok1'):
        """ Autoryzacja użytkownika do testów. Jeśli użytkownik o padanej nazwie nie istnieje, zostaje utworzony"""

        if not User.objects.filter(username=self.users[user]['username']):
            self.user = self.create_user(user)
        else:
            self.user = User.objects.get(username=self.users[user]['username'])
        self.client.force_login(self.user)

    def authorize_client(self, user=None, client=None,):
        if not user: self.user = self.create_user()
        else: self.user = user
        if not client: self.clientOb = self.create_client(self.user)
        else: self.clientOb = client
        session = self.client.session
        session['client_authorized'] = {'phone': self.clientOb.phone_number, 'user':self.user.username}
        session.save()

    def create_user(self, user='ok1'):
        return User.objects.create_user(username=self.users[user]['username'],
                                        password=self.users[user]['password'],
                                        is_active=self.users[user]['is_active'])

    def user_reverse(self, username):
        for user in self.users.keys():
            if username == self.users[user]['username']:
                return user

    def create_client(self, user, client='cl_ok1'):
        user_short = self.user_reverse(user.username)
        return ClientModel.objects.create(user=user,
                                          phone_number=self.clients[user_short][client]['phone_number'],
                                          email=self.clients[user_short][client]['email'],
                                          name=self.clients[user_short][client]['name'],
                                          surname=self.clients[user_short][client]['surname'],
                                          description=self.clients[user_short][client]['description'],
                                          pin=self.clients_full[user_short][client]['pin'],
                                          is_active=self.clients_full[user_short][client]['is_active'])


    def create_service(self, user, service='serv_ok1'):
        user_short = self.user_reverse(user.username)
        return Service.objects.create(user=user,
                                          name=self.services[user_short][service]['name'],
                                          duration=self.services[user_short][service]['duration'],
                                          is_active=self.services[user_short][service]['is_active'])
