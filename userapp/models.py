from datetime import time, timedelta
from django.contrib.auth.models import User
from django.core.exceptions import ValidationError
from django.db import models
from django.db.models import Q
from django.db.models.signals import post_save
from django.dispatch import receiver
from django.urls import reverse
from django.utils import timezone

class Client(models.Model):

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
        return reverse('dashboard_clients_remove', args=[self.id])



class Service(models.Model):

    """ User create services. Then his client can choose one of services and book visit """

    user = models.ForeignKey(User, default ='', on_delete=models.CASCADE)
    name = models.CharField(max_length=60, blank=False, default='')
    duration = models.DurationField(blank=False, default='')
    is_active = models.BooleanField(default=True)

    class Meta:
        # One user cannot add two services with the same name
        unique_together = ('user', 'name')

    def get_remove_url(self):
        return reverse('dashboard_settings_service_remove', args=[self.id])

    def get_lock_url(self):
        return reverse('dashboard_settings_service_lock', args=[self.id])

    def display_duration(self):
        # Duration should be display in format: 00:00.
        # TODO: Sprawdź czy istnieje funkcja która robi to bardziej elegancko
        return str(self.duration)[:-3].rjust(5,'0')

    def clean(self):
        if not self.duration:
            raise ValidationError('Wybierz czas')

class Visit(models.Model):

    """ Visits can be created by user or by client. If client do it, user need to confirmed visit (is_confirmed -> True)
     Client can cancel visit. if he do it before confirmation, visit is deleted. If after, user need to confirm it
     to delete"""

    user = models.ForeignKey(User, default ='', on_delete=models.CASCADE)
    client = models.ForeignKey(Client, default ='', on_delete=models.CASCADE)
    name = models.CharField(max_length=60, blank=False, default='')
    start = models.DateTimeField(auto_now=False, auto_now_add=False)
    end = models.DateTimeField(auto_now=False, auto_now_add=False)
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

        for visit in current_visits:
            if visit.start <= self.start < visit.end or visit.start < self.end <= visit.end:
                raise ValidationError('Termin zajęty')

    def get_confirm_url(self):
        return reverse('dashboard_visit_confirm', args=[self.id])

    def get_reject_url(self):
        return reverse('dashboard_visit_reject', args=[self.id])

    def get_cancel_url(self):
        return reverse('client_app_cancel_visit', args=[self.user, self.id])


class WorkTime(models.Model):

    user = models.OneToOneField(User, on_delete=models.CASCADE)
    start_monday = models.TimeField(auto_now=False, auto_now_add=False, default=time(8,0))
    duration_monday = models.DurationField(blank=False, default=timedelta(hours=8))
    start_tuesday = models.TimeField(auto_now=False, auto_now_add=False, default=time(8,0))
    duration_tuesday = models.DurationField(blank=False, default=timedelta(hours=8))
    start_wednesday = models.TimeField(auto_now=False, auto_now_add=False, default=time(8,0))
    duration_wednesday = models.DurationField(blank=False, default=timedelta(hours=8))
    start_thursday = models.TimeField(auto_now=False, auto_now_add=False, default=time(8,0))
    duration_thursday = models.DurationField(blank=False, default=timedelta(hours=8))
    start_friday = models.TimeField(auto_now=False, auto_now_add=False, default=time(8,0))
    duration_friday = models.DurationField(blank=False, default=timedelta(hours=8))
    start_saturday = models.TimeField(auto_now=False, auto_now_add=False, default=time(8,0))
    duration_saturday = models.DurationField(blank=False, default=timedelta(hours=8))
    start_sunday = models.TimeField(auto_now=False, auto_now_add=False, default=time(8,0))
    duration_sunday = models.DurationField(blank=False, default=timedelta(hours=8))
    holidays = models.BooleanField(default=False)
    earliest_visit = models.PositiveIntegerField(default=1, null=False)
    latest_visit = models.PositiveIntegerField(default=14, null=False)

    def clean(self):
        errors = []
        #if self.duration <= timedelta(minutes=0):
            #errors.append('Popraw godziny pracy')
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
            WorkTime.objects.create(user=instance)