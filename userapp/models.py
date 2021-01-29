from datetime import time, timedelta, datetime
from django.conf import settings
from django.contrib.auth.models import User
from django.core.exceptions import ValidationError
from django.db import models
from django.db.models import Q
from django.db.models.signals import post_save
from django.dispatch import receiver
from django.urls import reverse
from django.utils import timezone


def td_to_string(td):
    hours = str(td.days*24 + td.seconds//3600).rjust(2,'0')
    minutes = str((td.seconds//60)%60).rjust(2,'0')
    string = f'{hours}:{minutes}'
    return string


class Client(models.Model):
    def __str__(self):
        return self.name

    """ Client is different model than user. User is owner of clients and can create themm """

    def only_digits(value):
        if value.isdigit() == False:
            raise ValidationError('Podaj prawidłowy numer telefonu')

    user = models.ForeignKey(User, default ='', on_delete=models.CASCADE)
    phone_number = models.CharField(blank=False, max_length=20, validators=[only_digits])
    email = models.EmailField(max_length=40, blank=True, default='')
    name = models.CharField(max_length=20, blank=False, default='')
    surname = models.CharField(max_length=40, blank=True, default='')
    description = models.CharField(max_length=200, blank=True, default='')
    pin = models.CharField(max_length=4, blank=False, default='')
    is_active = models.BooleanField(default=True)

    class Meta:
        # One user cannot add two clients with the same phone number, but client
        # with one number should be added to shedules of two users
        unique_together = ('user', 'phone_number')

    def get_remove_url(self):
        return reverse('clients_remove', args=[self.id])

class Service(models.Model):
    def __str__(self):
        return self.name

    """ User create services. Then his client can choose one of services and book visit """

    user = models.ForeignKey(User, default ='', on_delete=models.CASCADE)
    name = models.CharField(max_length=60, blank=False, default='')
    duration = models.DurationField(blank=False, default='')
    is_active = models.BooleanField(default=True)

    class Meta:
        # One user cannot add two services with the same name
        unique_together = ('user', 'name')

    def get_remove_url(self):
        return reverse('settings_services_remove', args=[self.id])

    def get_lock_url(self):
        return reverse('settings_services_lock', args=[self.id])

    def display_duration(self):
        # Duration should be display in format: 00:00.
        return td_to_string(self.duration)

    def clean(self):
        if not self.duration:
            raise ValidationError('Ustaw czas')

class Visit(models.Model):
    def __str__(self):
        return self.name

    """ Visits can be created by user or by client. If client do it, user need to confirmed visit (is_confirmed -> True)
     Client can cancel visit. if he do it before confirmation, visit is deleted. If after, user need to confirm it
     to delete"""

    user = models.ForeignKey(User, default ='', on_delete=models.CASCADE)
    client = models.ForeignKey(Client, default ='', on_delete=models.CASCADE)
    name = models.CharField(max_length=60, blank=False, default='')
    start = models.DateTimeField(auto_now=False, auto_now_add=False, default=timezone.now)
    end = models.DateTimeField(auto_now=False, auto_now_add=False, default=timezone.now)
    is_available = models.BooleanField(default=True)
    is_confirmed = models.BooleanField(default=False)
    description = models.CharField(max_length=400, blank=True, default='')


    def clean(self):
        errors = []
        start_d = self.start.date()
        end_d = self.end.date()

        if self.end.date() != self.start.date():
            current_visits = Visit.objects.filter(Q(user=self.user, client=self.client, start__year=start_d.year, start__month=start_d.month, start__day=start_d.day) |
                                                  Q(user=self.user, client=self.client, end__year=start_d.year, end__month=start_d.month, end__day=start_d.day) |
                                                  Q(user=self.user, client=self.client, start__year=end_d.year, start__month=end_d.month, start__day=end_d.day) |
                                                  Q(user=self.user, client=self.client, end__year=end_d.year, end__month=end_d.month, end__day=end_d.day))
        else:
            current_visits = Visit.objects.filter(Q(user=self.user, client=self.client, start__year=start_d.year, start__month=start_d.month, start__day=start_d.day) |
                                                  Q(user=self.user, client=self.client, end__year=start_d.year, end__month=start_d.month, end__day=start_d.day))

        if self.start >= self.end:
            raise ValidationError('Czas trwania usługi jest zbyt krótki')
        if (self.end - self.start) > timedelta(hours=8):
            raise ValidationError('Usługa może trwać maksymalnie 8h')
        if self.start.time().minute % 15 != 0 or self.end.time().minute % 15 != 0:
            raise ValidationError('Nie kombinuj bo zepsujesz. Czas musi być podzielny przez 15 min')
        for visit in current_visits:
            if visit.start <= self.start < visit.end or visit.start < self.end <= visit.end or self.start < visit.start < self.end:
                if visit.is_available or (not visit.is_available and not visit.is_confirmed):
                    raise ValidationError('Termin zajęty')

    def get_display_url(self):
        return reverse('schedule_visit', args=[self.id])

    def get_start_date(self):
        return self.start.strftime('%y-%m-%d')

    def get_start_time(self):
        return self.start.strftime('%H:%M')

    def get_duration(self):
        return td_to_string(self.end - self.start)

    def if_past(self):
        if self.end < datetime.now():
            return True



class WorkTime(models.Model):

    user = models.ForeignKey(User, default ='', on_delete=models.CASCADE)
    day_of_week = models.SmallIntegerField(blank=False, default=0)
    start = models.DurationField(blank=False, default='')
    end = models.DurationField(blank=False, default='')

    def clean(self):
        errors = []
        if self.end <= self.start:
            errors.append('Popraw godziny pracy')
        if 0 > self.day_of_week > 6:
            errors.append('Nie kombinuj!')

        errors = ', '.join(errors)
        if errors:
            raise ValidationError(errors)

    def get_remove_url(self):
        return reverse('settings_work_time_remove', args=[self.id])

class UserSettings(models.Model):

    def only_digits(value):
        if value.isdigit() == False:
            raise ValidationError('Podaj prawidłowy numer telefonu')

    user = models.OneToOneField(settings.AUTH_USER_MODEL, on_delete=models.CASCADE)
    site_url = models.CharField(max_length=25, unique=True, blank=True)
    site_name = models.CharField(max_length=60, blank=True)
    phone_number = models.CharField(max_length=20, validators=[only_digits], blank=True)
    holidays = models.BooleanField(default=False)
    earliest_visit = models.PositiveIntegerField(default=1, null=False)
    latest_visit = models.PositiveIntegerField(default=14, null=False)

    def clean(self):
        errors = []
        if self.earliest_visit >= self.latest_visit:
            errors.append('Popraw możliwość wyboru terminów')
        if self.earliest_visit < 0 or self.latest_visit < 0:
            errors.append('Popraw możliwość wyboru terminów')

        errors = ', '.join(errors)
        if errors:
            raise ValidationError(errors)

    @receiver(post_save, sender=User)
    def create_profile(sender, instance, created, **kwargs):
        if created:
            UserSettings.objects.create(user=instance, site_url=instance.username, site_name=instance.username.title())

class TimeOff(models.Model):

    user = models.ForeignKey(User, default ='', on_delete=models.CASCADE)
    start = models.DateTimeField(auto_now=False, auto_now_add=False, default=timezone.now)
    end = models.DateTimeField(auto_now=False, auto_now_add=False, default=timezone.now)