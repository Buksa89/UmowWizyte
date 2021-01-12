from datetime import date, datetime, timedelta
from django import forms
from django.contrib.auth.models import User
from django.core.exceptions import NON_FIELD_ERRORS, ValidationError
from django.forms import Textarea
from django.shortcuts import get_object_or_404
from .models import Client, Service, Visit, UserSettings, WorkTime
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
    username = forms.CharField(label='Login', error_messages={'required': 'Podaj login'})
    password = forms.CharField(widget=forms.PasswordInput, label='Hasło',
                               error_messages={'required': 'Podaj hasło'})

    def clean(self):
        cd = self.cleaned_data
        if User.objects.filter(username=cd['username'], is_active=False):
            raise forms.ValidationError('Konto zablokowane')


class AddClientForm(forms.ModelForm):


    def save(self, user):
        self.instance.user = user
        self.instance.pin = pin_generate()
        return super().save()



    class Meta:
        model = Client
        fields = ['phone_number', 'name', 'surname', 'description']
        labels = {
            'name': 'Imię*',
            'surname': 'Nazwisko',
            'description': 'Dodatkowe info',
            'phone_number': 'Telefon*',
        }
        widgets = {
            'name': forms.fields.TextInput(attrs={'placeholder': 'Imię (to pole wyświetla sie klientowi!)',}),
            'surname': forms.fields.TextInput(attrs={'placeholder': 'Nazwisko',}),
            'description': forms.fields.TextInput(attrs={'placeholder': 'Opis (Tego pola klient nie widzi',}),
            'phone_number': forms.TextInput(attrs={'placeholder': 'Telefon'}),
            }
        error_messages = {
            'name': {'required': "Pole nie może być puste",
                     'max_length': 'Nazwa jest za długa'},
            'surname': {'max_length': 'Nazwisko jest za długie'},
            'description': {'max_length': 'Opis jest za długi'},
            'phone_number': {'required': 'Pole nie może być puste',
                             'max_length': 'Numer jest za długi'},
            NON_FIELD_ERRORS: {
                'unique_together': "Posiadasz już klienta z tym numerem telefonu",
            }
        }

class EditClientForm(forms.ModelForm):

    class Meta:
        model = Client
        fields = ['phone_number', 'name', 'surname', 'description', 'is_active']
        labels = {
            'name': 'Imię*',
            'surname': 'Nazwisko',
            'description': 'Dodatkowe info',
            'phone_number': 'Telefon*',
            'is_active': 'Aktywny',
        }
        widgets = {
            'name': forms.fields.TextInput(attrs={'placeholder': 'Imię (to pole wyświetla sie klientowi!)',}),
            'surname': forms.fields.TextInput(attrs={'placeholder': 'Nazwisko',}),
            'description': forms.fields.TextInput(attrs={'placeholder': 'Opis (Tego pola klient nie widzi',}),
            'phone_number': forms.TextInput(attrs={'placeholder': 'Telefon'}),
            }
        error_messages = {
            'name': {'required': "Pole nie może być puste",
                     'max_length': 'Nazwa jest za długa'},
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
            'name': 'Nazwa',
            'duration': 'Czas trwania',
            'is_active': 'Aktywna'
        }
        widgets = {
            'name': forms.fields.TextInput(attrs={'placeholder': 'Nazwa',}),
            'duration': forms.Select(choices=time_options(timedelta(hours=8), True)),
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

    end_monday = forms.ChoiceField(label='do:', choices=time_options(timedelta(hours=24), False))
    end_tuesday = forms.ChoiceField(label='do:', choices=time_options(timedelta(hours=24), False))
    end_wednesday = forms.ChoiceField(label='do:', choices=time_options(timedelta(hours=24), False))
    end_thursday = forms.ChoiceField(label='do:', choices=time_options(timedelta(hours=24), False))
    end_friday = forms.ChoiceField(label='do:', choices=time_options(timedelta(hours=24), False))
    end_saturday = forms.ChoiceField(label='do:', choices=time_options(timedelta(hours=24), False))
    end_sunday = forms.ChoiceField(label='do:', choices=time_options(timedelta(hours=24), False))

    class Meta:
        model = WorkTime
        fields = ['start_monday', 'end_monday', 'start_tuesday', 'end_tuesday', 'start_wednesday',
                  'end_wednesday', 'start_thursday', 'end_thursday', 'start_friday', 'end_friday',
                  'start_saturday', 'end_saturday', 'start_sunday', 'end_sunday', 'holidays',
                  'earliest_visit', 'latest_visit']

        labels = {
            'start_monday': 'od:',
            'start_tuesday': 'od:',
            'start_wednesday': 'od:',
            'start_thursday': 'od:',
            'start_friday': 'od:',
            'start_saturday': 'od:',
            'start_sunday': 'od:',
            'holidays': 'Pracuję w świeta',
            'earliest_visit': 'od:',
            'latest_visit': 'do:',
        }
        widgets = {
            'start_monday': forms.Select(choices=time_options(timedelta(hours=24), False)),
            'start_tuesday': forms.Select(choices=time_options(timedelta(hours=24), False)),
            'start_wednesday': forms.Select(choices=time_options(timedelta(hours=24), False)),
            'start_thursday': forms.Select(choices=time_options(timedelta(hours=24), False)),
            'start_friday': forms.Select(choices=time_options(timedelta(hours=24), False)),
            'start_saturday': forms.Select(choices=time_options(timedelta(hours=24), False)),
            'start_sunday': forms.Select(choices=time_options(timedelta(hours=24), False)),
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

    def save(self, durations):
        self.instance.duration_monday = durations[0]
        self.instance.duration_tuesday = durations[1]
        self.instance.duration_wednesday = durations[2]
        self.instance.duration_thursday = durations[3]
        self.instance.duration_friday = durations[4]
        self.instance.duration_saturday = durations[5]
        self.instance.duration_sunday = durations[6]
        return super().save()

class NewVisitForm(forms.Form):
    def __init__(self, *args, **kwargs):
        self.user = kwargs.pop('user')
        super(NewVisitForm, self).__init__(*args, **kwargs)
        self.fields['client'] = forms.ChoiceField(label='', choices=self.client_choices())
        self.fields['service'] = forms.ChoiceField(label='',  choices=self.service_choices())
        self.fields['duration'] = forms.ChoiceField(label='', choices=self.duration_choices())

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

    def duration_choices(self):
        choices = time_options(timedelta(hours=8))
        choices[0][1] = 'Standardowy czas'
        return choices


class AddVisitForm(forms.ModelForm):
    def save(self, user, client, name, start, end):
        self.instance.user = user
        self.instance.client = client
        self.instance.name = name
        self.instance.start = start
        self.instance.end = end
        self.instance.is_available = True
        self.instance.is_confirmed = True
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

class RegistrationForm(forms.ModelForm):
    password = forms.CharField(label='I hasło',
                               widget=forms.PasswordInput)
    password2 = forms.CharField(label='Dla pewności powtórz hasło',
                                widget=forms.PasswordInput)
    class Meta:
        model = User
        fields = ('username', 'first_name', 'last_name', 'email')
        labels = {
            'username': 'Twoja nazwa użytkownika:',
            'first_name': 'Jak masz na imię?',
            'last_name': 'I na nazwisko?',
            'email': 'Podaj jeszcze adres email',
        }

    def clean_password2(self):
        cd = self.cleaned_data
        if cd['password'] != cd['password2']:
            raise forms.ValidationError('Hasła się różnią')
        return cd['password2']

class ContactForm(forms.Form):

    content = forms.CharField(widget=forms.Textarea(attrs={'placeholder': 'Hej, jeśli masz problem z aplikacją lub masz pomysł jak ją ulepszyć, daj mi znać!'}), label='', )

class UserEditForm(forms.ModelForm):
    class Meta:
        model = User
        fields = ['first_name', 'last_name', 'email']

class UserSettingsForm(forms.ModelForm):
    class Meta:
        model = UserSettings
        fields = ['site_name', 'site_url']

class UserPassForm(forms.ModelForm):
    password = forms.CharField(widget=forms.PasswordInput, label='Hasło', required=False)
    password2 = forms.CharField(widget=forms.PasswordInput, label='Powtórz hasło', required=False)

    class Meta:
        model = User
        fields = ('password',)

    def clean_password2(self):
        cd = self.cleaned_data
        if cd['password'] != cd['password2']:
            raise forms.ValidationError('Hasła się różnią')
        return cd['password2']