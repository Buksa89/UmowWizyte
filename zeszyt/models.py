from django.contrib.auth.models import User
from django.core.exceptions import ValidationError
from django.db import models
from django.db.models.signals import post_save
from django.dispatch import receiver
from django.urls import reverse

class Client(models.Model):
    """ Client is different model than user. User can create clients. then he is owner of them"""
    user = models.ForeignKey(User, default ='', on_delete=models.CASCADE)
    phone_number = models.CharField(blank=False, max_length=20)
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
        return str(self.duration)[-5:]


class Visit(models.Model):
    user = models.ForeignKey(User, default ='', on_delete=models.CASCADE)
    client = models.ForeignKey(Client, default ='', on_delete=models.CASCADE)
    name = models.CharField(max_length=60, blank=False, default='')
    start = models.TimeField(auto_now=False, auto_now_add=False)
    stop = models.TimeField(auto_now=False, auto_now_add=False)
    is_avaible = models.BooleanField(default=True)
    is_confirmed = models.BooleanField(default=False)
    description = models.CharField(max_length=200, blank=True, default='')

    def get_remove_url(self):
        return reverse('remove_visit', args=[self.id])

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
        # TODO: Błędy powinny móc pojawić się jednocześnie
        if self.start_time >= self.end_time:
            raise ValidationError('Popraw godziny pracy')
        if self.earliest_visit >= self.latest_visit:
            raise ValidationError('Popraw możliwość wyboru terminów')

    @receiver(post_save, sender=User)
    def create_profile(sender, instance, created, **kwargs):
        if created:
            WorkTime.objects.create(user=instance)