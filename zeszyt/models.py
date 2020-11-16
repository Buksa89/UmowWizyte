from django.contrib.auth.models import User
from django.db import models
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