from django.contrib.auth.models import User
from django.test import TestCase
from django.test import Client
from ..models import Client as ClientModel

class BaseTest(TestCase):

    def setUp(self):
        self.users = {'ok1': {'username':'Rzęsiście1','password': 'RzęsPass12', 'is_active':True},
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

        self.services = {'ok1': {
                            'serv_ok1': {
                                'name': 'Lifting1',
                                'duration': '01:00',
                                'is_active': True,
                            },
                            'serv_ok2': {
                                'name': 'Doklejanie2',
                                'duration': '02:30',
                                'is_active': True,
                            },
                            'serv_ok3': {
                                'name': 'Paznokcie3',
                                'duration': '00:15',
                                'is_active': True,
                            },
                            'serv_ok4': {
                                'name': 'Pogaduchy4',
                                'duration': '05:00',
                                'is_active': True,
                            },
                            'serv_not_active': {
                                'name': 'SpankoNA',
                                'duration': '01:00',
                                'is_active': False,
                            }
                        },
                         'ok2': {
                            'serv_ok1': {
                                'name': 'Golenie1',
                                'duration': '00:30',
                                'is_active': True,
                            },
                            'serv_ok2': {
                                'name': 'Wycinka drzewa2',
                                'duration': '02:30',
                                'is_active': True,
                            },
                            'serv_ok3': {
                                'name': 'Herbatka3',
                                'duration': '00:15',
                                'is_active': True,
                            },
                            'serv_ok4': {
                                'name': 'Polowanie na niedźwiedzia4',
                                'duration': '08:00',
                                'is_active': True,
                            },
                            'serv_not_active': {
                                'name': 'Ucieczka przed niedźwiedziemNA',
                                'duration': '03:00',
                                'is_active': False,
                            }
                        }
                        }

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