from django.contrib.auth.models import User
from django.contrib.staticfiles.testing import StaticLiveServerTestCase
from selenium import webdriver
from selenium.common.exceptions import WebDriverException
import time
from zeszyt.models import Client
MAX_WAIT = 5

class FunctionalTest(StaticLiveServerTestCase):
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

        self.browser = webdriver.Firefox()
        self.add_some_users()
        self.browser.get(self.live_server_url+'/panel')





    def tearDown(self):
        self.browser.quit()


    """ Funkcje pomocnicze """
    def wait_for(self, fn):
        start_time = time.time()
        while True:
            try:
                return fn()
            except (AssertionError, WebDriverException) as e:
                if time.time() - start_time > MAX_WAIT:
                    raise e
                time.sleep(0.5)

    def send_form(self, field):
        for field_name, field_value in field.items():
            find_field = self.browser.find_element_by_name(field_name)
            try:  find_field.clear()
            except:
                if find_field.get_attribute("type") == 'checkbox':
                    if  field_value and not find_field.is_selected():
                        find_field.click()
                    elif not field_value and find_field.is_selected():
                        find_field.click()
                elif find_field.get_attribute("type") == 'select-one':
                    for option in find_field.find_elements_by_tag_name('option'):
                        if option.text == field_value:
                            option.click()
                            break
            self.browser.find_element_by_name(field_name).send_keys(field_value)
        self.browser.find_element_by_name('submit').click()


    def add_some_users(self):
        """funkcja tworzy przykłądowych użytkoników """

        for name, data in self.users.items():
            User.objects.create_user(username=data['username'],
                                           password=data['password'],
                                           is_active=data['is_active'])

    def login_user(self, user):
        field = {'username': self.users[user]['username'], 'password': self.users[user]['password']}
        self.send_form(field)

    def get_pin(self, number, user):
        user = User.objects.get(username=user)
        return Client.objects.get(phone_number=number, user=user).pin

    def get_new_url(self, old, new):
        return self.browser.current_url.replace(old, new)

    def add_some_clients(self):
        """ Funkcja tworzy klientów dla użytkowników oznaczonych ok1 i ok2"""

        for user in self.users.keys():
            if user == 'not_active': break;
            self.wait_for(lambda: self.login_user(user))
            self.wait_for(lambda: self.browser.find_element_by_link_text('Klienci').click())
            self.wait_for(lambda: self.browser.find_element_by_link_text('Dodaj klienta').click())
            for client in self.clients[user].keys():
                self.send_form(self.clients[user][client])
                self.clients[user][client]['user'] = self.users[user]['username']
                self.clients[user][client]['pin'] = self.get_pin(self.clients[user][client]['phone_number'],
                                                                 self.clients[user][client]['user'])
                #TODO: Deaktywuj niektórych klientów
            self.wait_for(lambda: self.browser.find_element_by_link_text('Wyloguj').click())