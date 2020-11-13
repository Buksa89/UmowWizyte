from django.contrib.auth.models import User
from django.db import models
from django.urls import reverse

class Client(models.Model):
    user = models.ForeignKey(User, default ='', on_delete=models.CASCADE)
    phone_number = models.CharField(blank=False, max_length=20)
    email = models.EmailField(max_length=40, blank=True, default='')
    name = models.CharField(max_length=20, blank=False, default='')
    surname = models.CharField(max_length=40, blank=True, default='')
    description = models.CharField(max_length=200, blank=True, default='')
    pin = models.CharField(max_length=4, blank=False, default='')
    is_active = models.BooleanField(default=True)

    class Meta:
        # Jeden użytkownik nie powiniene móc dodać dwóch klientów o tym samym numerze telefonu, al
        # klient o podanym numerze telefonu powinien móc logować się na harmonogramy różnych użytkowników
        unique_together = ('user', 'phone_number')


    def get_remove_url(self):
        return reverse('remove_client_screen', args=[self.id])


class Service(models.Model):
    user = models.ForeignKey(User, default ='', on_delete=models.CASCADE)
    name = models.CharField(max_length=60, blank=False, default='')
    duration = models.DurationField(blank=False, default='')
    is_active = models.BooleanField(default=True)
    class Meta:
        # Jeden użytkownik nie powinien mieć możliwości dodania dwóch usług o tej samej nazwie
        unique_together = ('user', 'name')

    def get_remove_url(self):
        return reverse('remove_service', args=[self.id])

    def display_duration(self):
        return str(self.duration)[-5:]