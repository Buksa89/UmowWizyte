from datetime import datetime, timedelta
from django import forms
from django.contrib.auth.models import User
from django.core.exceptions import NON_FIELD_ERRORS, ValidationError
from django.shortcuts import get_object_or_404
from .models import Client, Service, Visit, WorkTime
from .base import pin_generate, time_options


class only_digits (object):
    #TODO: Czy ta funkcja jest jeszcze gdziekolwiek uzywana?
    def __init__(self, message_type):
        self.message = ''
        if message_type == 'phone': self.message = 'Podaj prawidłowy numer telefonu'
        elif message_type == 'pin': self.message = 'Podaj prawidłowy pin'

    def __call__(self, value):
        if value.isdigit() == False:
            raise ValidationError(self.message)

class LoginForm(forms.Form):
    username = forms.CharField(label='Imię', error_messages={'required': 'Podaj login'})
    password = forms.CharField(widget=forms.PasswordInput, label='Hasło',
                               error_messages={'required': 'Podaj hasło'})

class AddClientForm(forms.ModelForm):


    def save(self, user):
        self.instance.user = user
        self.instance.pin = pin_generate()
        return super().save()



    class Meta:
        model = Client
        fields = ['phone_number', 'email', 'name', 'surname', 'description']
        labels = {
            'email': 'email',
            'name': 'Imię*',
            'surname': 'Nazwisko',
            'description': 'Dodatkowe info',
            'phone_number': 'Telefon*',
        }
        widgets = {
            'email': forms.fields.TextInput(attrs={'placeholder': 'Mail',}),
            'name': forms.fields.TextInput(attrs={'placeholder': 'Imię (to pole wyświetla sie klientowi!)',}),
            'surname': forms.fields.TextInput(attrs={'placeholder': 'Nazwisko',}),
            'description': forms.fields.TextInput(attrs={'placeholder': 'Opis (Tego pola klient nie widzi',}),
            'phone_number': forms.TextInput(attrs={'placeholder': 'Telefon'}),
            }
        error_messages = {
            'name': {'required': "Pole nie może być puste",
                     'max_length': 'Nazwa jest za długa'},
            'email': {'invalid': 'Email nieprawidłowy','max_length': 'Email jest za długi'},
            'surname': {'max_length': 'Nazwisko jest za długie'},
            'description': {'max_length': 'Opis jest za długi'},
            'phone_number': {'required': 'Pole nie może być puste',
                             'max_length': 'Numer jest za długi'},
            NON_FIELD_ERRORS: {
                'unique_together': "Posiadasz już klienta z tym numerem telefonu",
            }
        }

class AddServiceForm(forms.ModelForm):

    class Meta:
        model = Service
        fields = ['name', 'duration', 'is_active']
        labels = {
            'name': '',
            'duration': '',
            'is_active': ''
        }
        widgets = {
            'name': forms.fields.TextInput(attrs={'placeholder': 'Nazwa',}),
            'duration': forms.Select(choices=time_options(8, True)),
            'is_active': forms.CheckboxInput()
            }

        error_messages = {
            'duration': {'required': "Pole nie może być puste",
                             'invalid': "Nie kombinuj!"},

            'name': {'required': "Pole nie może być puste",
                     'max_length': "Nazwa jest za długa"},

            NON_FIELD_ERRORS: {
                'unique_together': "Usługa o tej nazwie już istnieje",
            }
        }

    def save(self, user):
        self.instance.user = user
        return super().save()

class WorkTimeForm(forms.ModelForm):


    class Meta:
        model = WorkTime
        fields = ['start_time', 'end_time', 'monday', 'tuesday', 'wednesday', 'thursday',
                  'friday', 'saturday', 'sunday', 'holidays', 'earliest_visit', 'latest_visit']
        labels = {
            'start_time': 'Od godziny',
            'end_time': 'Do godziny',
            'monday': 'Poniedziałki',
            'tuesday': 'Wtorki',
            'wednesday': 'Środy',
            'thursday': 'Czwartki',
            'friday': 'Piątki',
            'saturday': 'Soboty',
            'sunday': 'Niedziele',
            'holidays': 'Święta',
            'earliest_visit': 'Terminy wizyt (za ile dni najwcześniej)',
            'latest_visit': 'Terminy wizyt (za ile dni najpóźniej)',
        }
        widgets = {
            'start_time': forms.Select(choices=time_options(24, False)),
            'end_time': forms.Select(choices=time_options(24, False)),
            'monday': forms.CheckboxInput(),
            'tuesday': forms.CheckboxInput(),
            'wednesday': forms.CheckboxInput(),
            'thursday': forms.CheckboxInput(),
            'friday': forms.CheckboxInput(),
            'saturday': forms.CheckboxInput(),
            'sunday': forms.CheckboxInput(),
            'holidays': forms.CheckboxInput(),
            'earliest_visit': forms.NumberInput(),
            'latest_visit': forms.NumberInput(),
            }
        error_messages = {
            'earliest_visit': {'required': "Pole nie może być puste",
                               'min-value':"Liczba dni nieprawidłowa"},

            'latest_visit': {'required': "Pole nie może być puste",
                             'invalid':"Liczba dni nieprawidłowa"},
        }





class AddVisitForm(forms.Form):
    def __init__(self, *args, **kwargs):
        self.user = kwargs.pop('user')
        super(AddVisitForm, self).__init__(*args, **kwargs)
        self.fields['client'] = forms.ChoiceField(label='', choices=self.client_choices())
        self.fields['service'] = forms.ChoiceField(label='',  choices=self.service_choices())
        self.fields['duration'] = forms.ChoiceField(label='', choices=time_options(8))

    def client_choices(self):
        clients = Client.objects.filter(user=self.user)
        choices = []
        for client in clients:
            choices.append([client.id, f'{client.name} {client.surname}'])
        return choices

    def service_choices(self):
        services = Service.objects.filter(user=self.user)
        choices = []
        for service in services:
            choices.append([service.id, f'{service.name} {service.display_duration()}'])
        return choices
