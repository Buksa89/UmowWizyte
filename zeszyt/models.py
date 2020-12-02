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

    def display_duration(self):
        # Duration should be display in format: 00:00.
        # TODO: Sprawdź czy istnieje funkcja która robi to bardziej elegancko
        return str(self.duration)[:-3].rjust(5,'0')


class Visit(models.Model):

    """ Visits can be created by user or by client. If client do it, user need to confirmed visit (is_confirmed -> True)
     Client can cancel visit. if he do it before confirmation, visit is deleted. If after, user need to confirm it
     to delete"""

    user = models.ForeignKey(User, default ='', on_delete=models.CASCADE)
    client = models.ForeignKey(Client, default ='', on_delete=models.CASCADE)
    name = models.CharField(max_length=60, blank=False, default='')
    start = models.DateTimeField(auto_now=False, auto_now_add=False)
    stop = models.DateTimeField(auto_now=False, auto_now_add=False)
    is_available = models.BooleanField(default=True)
    is_confirmed = models.BooleanField(default=False)
    description = models.CharField(max_length=400, blank=True, default='')

    def get_remove_url(self):
        return reverse('remove_visit', args=[self.id])

    def clean(self):
        errors = []
        self.start = timezone.make_aware(self.start, timezone.get_default_timezone())
        self.stop = timezone.make_aware(self.stop, timezone.get_default_timezone())
        start_d = self.start.date()
        stop_d = self.stop.date()

        if self.stop.date() != self.start.date():
            current_visits = Visit.objects.filter(Q(user=self.user, client=self.client, start__year=start_d.year, start__month=start_d.month, start__day=start_d.day) |
                                                  Q(user=self.user, client=self.client, stop__year=start_d.year, stop__month=start_d.month, stop__day=start_d.day) |
                                                  Q(user=self.user, client=self.client, start__year=stop_d.year, start__month=stop_d.month, start__day=stop_d.day) |
                                                  Q(user=self.user, client=self.client, stop__year=stop_d.year, stop__month=stop_d.month, stop__day=stop_d.day))
        else:
            current_visits = Visit.objects.filter(Q(user=self.user, client=self.client, start__year=start_d.year, start__month=start_d.month, start__day=start_d.day) |
                                                  Q(user=self.user, client=self.client, stop__year=start_d.year, stop__month=start_d.month, stop__day=start_d.day))

        for visit in current_visits:
            if visit.start <= self.start < visit.stop or visit.start < self.stop <= visit.stop:
                raise ValidationError('Termin zajęty')


class WorkTime(models.Model):


    user = models.OneToOneField(User, on_delete=models.CASCADE)
    start_time = models.TimeField(auto_now=False, auto_now_add=False, default="8:00")
    end_time = models.TimeField(auto_now=False, auto_now_add=False, default="16:00")
    monday = models.BooleanField(default=True)
    tuesday = models.BooleanField(default=True)
    wednesday = models.BooleanField(default=True)
    thursday = models.BooleanField(default=True)
    friday = models.BooleanField(default=True)
    saturday = models.BooleanField(default=False)
    sunday = models.BooleanField(default=False)
    holidays = models.BooleanField(default=False)
    earliest_visit = models.PositiveIntegerField(default=1, null=False)
    latest_visit = models.PositiveIntegerField(default=14, null=False)

    def clean(self):
        errors = []
        if self.start_time >= self.end_time:
            errors.append('Popraw godziny pracy')
        if self.earliest_visit >= self.latest_visit:
            errors.append('Popraw możliwość wyboru terminów')

        errors = ', '.join(errors)
        if errors:
            raise ValidationError(errors)

    @receiver(post_save, sender=User)
    def create_profile(sender, instance, created, **kwargs):
        if created:
            WorkTime.objects.create(user=instance)