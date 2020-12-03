from django import forms
from django.core.exceptions import NON_FIELD_ERRORS, ValidationError
from django.forms import Textarea
from random import choice
from userapp.models import Client, Service, Visit, WorkTime


class only_digits (object):
    def __init__(self, message_type):
        self.message = ''
        if message_type == 'phone': self.message = 'Podaj prawidłowy numer telefonu'
        elif message_type == 'pin': self.message = 'Podaj prawidłowy pin'

    def __call__(self, value):
        if value.isdigit() == False:
            raise ValidationError(self.message)

class ClientLoginForm(forms.Form):

    phone_number = forms.CharField(validators=[only_digits('phone')], label='Telefon', error_messages={'required': 'Podaj numer telefonu'})
    pin = forms.CharField(validators=[only_digits('pin')], label='pin', error_messages={'required': 'Podaj pin'})

class ClientChooseVisitForm(forms.Form):

    def __init__(self, user):
        super(ClientChooseVisitForm, self).__init__()
        self.user = user
        self.fields['service'] = forms.ChoiceField(label='', choices=self.get_avaible_visits())

    def get_avaible_visits(self):
        services = Service.objects.filter(user=self.user, is_active=True)
        service_list = ()
        for service in services:
            service_list += ((service.id,service.name),)
        return service_list


class AddVisit(forms.ModelForm):
    def save(self, user, client, name, start, stop, is_available, is_confirmed):
        self.instance.user = user
        self.instance.client = client
        self.instance.name = name
        self.instance.start = start
        self.instance.stop = stop
        self.instance.is_available = is_available
        self.instance.is_confirmed = is_confirmed
        return super().save()

    class Meta:
        model = Visit
        fields = ['description']
        labels = {
            'description': 'Dodatkowe informacje',
        }
        widgets = {
            'description': Textarea(attrs={'cols': 80, 'rows': 2,
                'placeholder': 'Dodatkowe informacje',}),
            }
        error_messages = {
            'description': {'max_length': 'Opis jest za długi'},
        }