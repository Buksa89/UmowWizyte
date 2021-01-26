from datetime import date, datetime, timedelta
from django.contrib.auth.models import User
from django.test import TestCase
from django.test import Client
from django.utils import timezone
from ..models import Client as ClientModel
from ..models import Service, Visit




class BaseTest(TestCase):



    users = {'active_1': {'username':'Rzesiscie1','password': 'RzesPass12', 'is_active':True},
             'active_2': {'username':'Brodacz','password': 'BarPass12', 'is_active':True},
             'not_active': {'username':'LeniwyLeszek','password': 'Nieaktywny', 'is_active':False},
             }










    def correct_datetime(start, duration=False):
        if not duration:
            return timezone.make_aware(start, timezone.get_default_timezone())

    # (datetime.combine(date.today(), hour) + timedelta(minutes=15)).time()





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
    work_time = {'regular': {'start_monday': '8:00', 'end_monday': '16:00', 'duration_monday': timedelta(hours=8),
                             'start_tuesday': '8:00', 'end_tuesday': '16:00', 'duration_tuesday': timedelta(hours=8),
                             'start_wednesday': '8:00', 'end_wednesday': '16:00', 'duration_wednesday': timedelta(hours=8),
                             'start_thursday': '8:00', 'end_thursday': '16:00', 'duration_thursday': timedelta(hours=8),
                             'start_friday': '8:00', 'end_friday': '16:00', 'duration_friday': timedelta(hours=8),
                             'start_saturday': '8:00', 'end_saturday': '16:00', 'duration_saturday': timedelta(hours=8),
                             'start_sunday': '8:00', 'end_sunday': '16:00', 'duration_sunday': timedelta(hours=8),
                             'holidays': False, 'earliest_visit': 0, 'latest_visit': 14,
                             },
                 'early_bird': {'start_monday': '0:00', 'end_monday': '8:00', 'duration_monday': timedelta(hours=8),
                                'start_tuesday': '0:00', 'end_tuesday': '8:00', 'duration_tuesday': timedelta(hours=8),
                                'start_wednesday': '0:00', 'end_wednesday': '8:00', 'duration_wednesday': timedelta(hours=8),
                                'start_thursday': '0:00', 'end_thursday': '8:00', 'duration_thursday': timedelta(hours=8),
                                'start_friday': '0:00', 'end_friday': '8:00', 'duration_friday': timedelta(hours=8),
                                'start_saturday': '0:00', 'end_saturday': '8:00', 'duration_saturday': timedelta(hours=8),
                                'start_sunday': '0:00', 'end_sunday': '8:00', 'duration_sunday': timedelta(hours=8),
                                'holidays': False, 'earliest_visit': 0, 'latest_visit': 14,
                                },

                 'weekend_holidays_free': {'start_monday': '8:00', 'end_monday': '16:00', 'duration_monday': timedelta(hours=8),
                                           'start_tuesday': '8:00', 'end_tuesday': '16:00', 'duration_tuesday': timedelta(hours=8),
                                           'start_wednesday': '8:00', 'end_wednesday': '16:00', 'duration_wednesday': timedelta(hours=8),
                                           'start_thursday': '8:00', 'end_thursday': '16:00', 'duration_thursday': timedelta(hours=8),
                                           'start_friday': '8:00', 'end_friday': '16:00', 'duration_friday': timedelta(hours=8),
                                           'start_saturday': '0:00', 'end_saturday': '0:00', 'duration_saturday': timedelta(hours=0),
                                           'start_sunday': '0:00', 'end_sunday': '0:00', 'duration_sunday': timedelta(hours=0),
                                           'holidays': False, 'earliest_visit': 0, 'latest_visit': 1000,
                                           },

                 'middle_week_free_eb':{'start_monday': '0:00', 'end_monday': '0:00', 'duration_monday': timedelta(hours=0),
                                        'start_tuesday': '0:00', 'end_tuesday': '0:00', 'duration_tuesday': timedelta(hours=0),
                                        'start_wednesday': '0:00', 'end_wednesday': '0:00', 'duration_wednesday': timedelta(hours=0),
                                        'start_thursday': '0:00', 'end_thursday': '0:00', 'duration_thursday': timedelta(hours=0),
                                        'start_friday': '0:00', 'end_friday': '0:00', 'duration_friday': timedelta(hours=0),
                                        'start_saturday': '8:00', 'end_saturday': '16:00', 'duration_saturday': timedelta(hours=8),
                                        'start_sunday': '8:00', 'end_sunday': '16:00', 'duration_sunday': timedelta(hours=8),
                                        'holidays': True, 'earliest_visit': 0, 'latest_visit': 1000,
                                       },

                 'lazy_joe': {'start_monday': '12:00', 'end_monday': '14:00',
                                         'duration_monday': timedelta(hours=2),
                                         'start_tuesday': '12:00', 'end_tuesday': '14:00',
                                         'duration_tuesday': timedelta(hours=2),
                                         'start_wednesday': '12:00', 'end_wednesday': '14:00',
                                         'duration_wednesday': timedelta(hours=2),
                                         'start_thursday': '12:00', 'end_thursday': '14:00',
                                         'duration_thursday': timedelta(hours=2),
                                         'start_friday': '12:00', 'end_friday': '14:00',
                                         'duration_friday': timedelta(hours=2),
                                         'start_saturday': '12:00', 'end_saturday': '14:00',
                                         'duration_saturday': timedelta(hours=2),
                                         'start_sunday': '12:00', 'end_sunday': '14:00',
                                         'duration_sunday': timedelta(hours=2),
                                         'holidays': True, 'earliest_visit': 0, 'latest_visit': 1000,
                                         }
                }


    visits = {'short_1': {'name': 'vis_1', 'start': correct_datetime(datetime(2022, 10, 24, 12, 0)), 'end': correct_datetime(datetime(2022, 10, 24, 13, 0)), 'is_available':True, 'is_confirmed':True},
              'overlap_1': {'name': 'vis_2', 'start': correct_datetime(datetime(2022, 10, 24, 11, 15)), 'end': correct_datetime(datetime(2022, 10, 24, 12, 15)), 'is_available':True, 'is_confirmed':True},
              'overlap_2': {'name': 'vis_3', 'start': correct_datetime(datetime(2022, 10, 24, 12, 45)), 'end': correct_datetime(datetime(2022, 10, 24, 13, 45)), 'is_available':True, 'is_confirmed':True},
              'not_overlap_1': {'name': 'vis_4', 'start': correct_datetime(datetime(2022, 10, 24, 11, 0)), 'end': correct_datetime(datetime(2022, 10, 24, 12, 0)), 'is_available':True, 'is_confirmed':True},
              'not_overlap_2': {'name': 'vis_5', 'start': correct_datetime(datetime(2022, 10, 24, 13, 0)), 'end': correct_datetime(datetime(2022, 10, 24, 14, 0)), 'is_available':True, 'is_confirmed':True},

              'later': {'name': 'vis_5', 'start': correct_datetime(datetime(2022, 10, 31, 13, 0)), 'end': correct_datetime(datetime(2022, 10, 31, 17, 0)), 'is_available':True, 'is_confirmed':True},
              'old': {'name': 'old', 'start': correct_datetime(datetime(2011, 10, 24, 12, 0)), 'end': correct_datetime(datetime(2011, 10, 24, 13, 0)), 'is_available':True, 'is_confirmed':True},
              'confirmed_1': {'name': 'vis_7', 'start': correct_datetime(datetime(2022, 10, 24, 13, 0)), 'end': correct_datetime(datetime(2022, 10, 24, 14, 0)), 'is_available':True, 'is_confirmed':True},
              'confirmed_2': {'name': 'vis_8', 'start': correct_datetime(datetime(2022, 10, 24, 14, 0)), 'end': correct_datetime(datetime(2022, 10, 24, 15, 0)), 'is_available':True, 'is_confirmed':True},
              'not_confirmed': {'name': 'not_confirmed', 'start': correct_datetime(datetime(2022, 10, 24, 15, 0)), 'end': correct_datetime(datetime(2022, 10, 24, 16, 0)), 'is_available':True, 'is_confirmed':False},
              'cancelled_confirmed': {'name': 'cancelled_confirmed', 'start': correct_datetime(datetime(2022, 10, 24, 16, 0)), 'end': correct_datetime(datetime(2022, 10, 24, 17, 0)), 'is_available':False, 'is_confirmed':True},
              'cancelled_not_confirmed': {'name': 'cancelled_not_confirmed', 'start': correct_datetime(datetime(2022, 10, 24, 17, 0)), 'end': correct_datetime(datetime(2022, 10, 24, 18, 0)), 'is_available':False, 'is_confirmed':False},

              }

    date_time = {'regular': datetime(2022, 10, 10, 8, 0, 0),
                }

    weeks = {'with_holiday': {'week': 44, 'year': 2022},
             'no_holiday': {'week': 43, 'year': 2022},
              }

    ANY_PHONE = 999999999
    ANY_PIN = 9999
    EMPTY = ''
    WRONG_DATA = 'wrong_data'
    CORRECT_PASSWORD = 'Qwerty123'
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

    def work_time_save(self, worktime, data):
        worktime.start_monday = self.work_time[data]['start_monday']
        worktime.duration_monday = self.work_time[data]['duration_monday']
        worktime.start_tuesday = self.work_time[data]['start_tuesday']
        worktime.duration_tuesday = self.work_time[data]['duration_tuesday']
        worktime.start_wednesday = self.work_time[data]['start_wednesday']
        worktime.duration_wednesday = self.work_time[data]['duration_wednesday']
        worktime.start_thursday = self.work_time[data]['start_thursday']
        worktime.duration_thursday = self.work_time[data]['duration_thursday']
        worktime.start_friday = self.work_time[data]['start_friday']
        worktime.duration_friday = self.work_time[data]['duration_friday']
        worktime.start_saturday = self.work_time[data]['start_saturday']
        worktime.duration_saturday = self.work_time[data]['duration_saturday']
        worktime.start_sunday = self.work_time[data]['start_sunday']
        worktime.duration_sunday = self.work_time[data]['duration_sunday']
        worktime.holidays = self.work_time[data]['holidays']
        worktime.earliest_visit = self.work_time[data]['earliest_visit']
        worktime.latest_visit = self.work_time[data]['latest_visit']
        worktime.save()


    def authorize_client(self, client):
        session = self.client.session
        session['client_authorized'] = {'phone': client.phone_number, 'user':client.user.username}
        session.save()

    def create_service(self, user, service='short_1'):
        return Service.objects.create(user=user,
                                          name=self.services[service]['name'],
                                          duration=self.services[service]['duration'],
                                          is_active=self.services[service]['is_active'])

    def create_visit(self, user, client, visit='short_1'):
        return Visit.objects.create(user=user,
                                    client = client,
                                    name = self.visits[visit]['name'],
                                    start = self.visits[visit]['start'],
                                    end = self.visits[visit]['end'],
                                    description = '',
                                    is_available = self.visits[visit]['is_available'],
                                    is_confirmed = self.visits[visit]['is_confirmed'])

