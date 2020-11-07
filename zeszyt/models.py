from django.contrib.auth.models import User
from django.db import models
from django.urls import reverse

class Client(models.Model):
    user = models.ForeignKey(User, default ='', on_delete=models.CASCADE)
    phone_number = models.PositiveIntegerField(blank=False)
    email = models.EmailField(max_length=40, blank=True, default='')
    name = models.CharField(max_length=20, blank=True, default='')
    surname = models.CharField(max_length=40, blank=True, default='')
    description = models.CharField(max_length=200, blank=True, default='')
    pin = models.CharField(max_length=4, blank=False, default='')
    next_pin = models.CharField(max_length=4, blank=False, default='')
    is_active = models.BooleanField(default=True)

    class Meta:
        # Klient o podanym numerze telefonu powinien móc logować się na harmonogramy różnych użytkowników
        unique_together = ('user', 'phone_number')


    def get_remove_url(self):
        return reverse('remove_client_screen', args=[self.id])