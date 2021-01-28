from datetime import date, datetime, timedelta
from django import forms
from django.contrib.auth.models import User
from django.core.exceptions import NON_FIELD_ERRORS, ValidationError
from django.forms import Textarea
from django.shortcuts import get_object_or_404
from .models import Client, Service, Visit, UserSettings, WorkTime
from .base import pin_generate, time_options
import userapp.language as l


"""class only_digits (object):
    #TODO: Czy ta funkcja jest jeszcze gdziekolwiek uzywana?
    def __init__(self, message_type):
        self.message = ''
        if message_type == 'phone': self.message = 'Podaj prawidłowy numer telefonu'
        elif message_type == 'pin': self.message = 'Podaj prawidłowy pin'

    def __call__(self, value):
        if value.isdigit() == False:
            raise ValidationError(self.message)"""

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
            'description': forms.fields.TextInput(attrs={'placeholder': 'Opis (Tego pola klient nie widzi)',}),
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
            'duration': forms.Select(choices=time_options(8)),
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

class NewWorkTimeForm(forms.ModelForm):
    days_of_week = (
        ("0", "Poniedziałek"),
        ("1", "Wtorek"),
        ("2", "Środa"),
        ("3", "Czwartek"),
        ("4", "Piątek"),
        ("5", "Sobota"),
        ("6", "Niedziela"),
    )

    day_of_week = forms.ChoiceField(label='Dzień Tygodnia', choices=days_of_week)
    start = forms.ChoiceField(label='do:', choices=time_options(24))
    end = forms.ChoiceField(label='do:', choices=time_options(24))

    class Meta:
        model = WorkTime
        fields = ['day_of_week', 'start', 'end']

    def save(self, user):
        self.instance.user = user
        return super().save()

class WorkHolidaysForm(forms.ModelForm):

    holidays = forms.BooleanField(label='Pracuję w świeta', widget=forms.CheckboxInput(), required=False)

    class Meta:
        model = UserSettings
        fields = ['holidays']

    def save(self, user):
        self.instance.user = user
        return super().save()

class NewVisitForm(forms.Form):
    def __init__(self, *args, **kwargs):
        self.user = kwargs.pop('user')
        super(NewVisitForm, self).__init__(*args, **kwargs)
        self.fields['client'] = forms.ChoiceField(label='Klient', choices=self.client_choices(), required=True)
        self.fields['service'] = forms.ChoiceField(label='Usługa',  choices=self.service_choices(), required=True)
        self.fields['duration'] = forms.ChoiceField(label='Czas trwania (Zostaw pole puste, jeśli usługa ma trwać tyle co zwykle)', choices=self.duration_choices())

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
        choices = time_options(8)
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

    username = forms.CharField(label=l.YOUR_USERNAME,
                                error_messages={'required': l.get_one(l.FIELD_REQUIRED,),
                                                'max_length': l.get_one(l.FIELD_TOO_LONG),},
                                max_length='40',)
    first_name = forms.CharField(label=l.YOUR_FIRST_NAME,
                                 error_messages={'required': l.get_one(l.FIELD_REQUIRED,),
                                                 'max_length': l.get_one(l.FIELD_TOO_LONG),},
                                max_length='40',)
    last_name = forms.CharField(label=l.YOUR_LAST_NAME,
                                error_messages={'required': l.get_one(l.FIELD_REQUIRED,),
                                                'max_length': l.get_one(l.FIELD_TOO_LONG),},
                                max_length='40',)
    email = forms.EmailField(required=True,
                             label=l.YOUR_EMAIL,
                             error_messages={'required': l.get_one(l.FIELD_REQUIRED,),
                                             'max_length': l.get_one(l.FIELD_TOO_LONG,),},
                             max_length='40',)
    password = forms.CharField(label=l.YOUR_PASSWORD,
                               widget=forms.PasswordInput,
                               error_messages={'required': l.get_one(l.FIELD_REQUIRED,),
                                               'max_length': l.get_one(l.FIELD_TOO_LONG),},
                               max_length='40',)
    password2 = forms.CharField(label=l.REPEAT_PASSWORD,
                                widget=forms.PasswordInput,
                                error_messages={'required': l.get_one(l.FIELD_REQUIRED,),
                                                'max_length': l.get_one(l.FIELD_TOO_LONG),},
                                max_length='40',)

    class Meta:
        model = User
        fields = ('username', 'first_name', 'last_name', 'email')

    def clean_email(self):
        email = self.cleaned_data['email']
        users = User.objects.filter(email__iexact=email)
        if users:
            raise forms.ValidationError(l.EMAIL_ALREADY_EXIST)
        return email.lower()

    def clean_username(self):
        username = self.cleaned_data['username']
        users = User.objects.filter(username__iexact=username)
        if users:
            raise forms.ValidationError(l.USERNAME_ALREADY_EXIST)
        return username

    def clean_password2(self):
        cd = self.cleaned_data
        if cd['password'] != cd['password2']:
            raise forms.ValidationError(l.PASSWORDS_DIFFERENT)
        if len(cd['password']) < 7 or \
                not any(char.isdigit() for char in cd['password']) or \
                not any(char.isupper() for char in cd['password']) or \
                not any(char.islower() for char in cd['password']):
            raise forms.ValidationError(l.PASSWORD_TOO_WEAK)
        return cd['password2']



class ContactForm(forms.Form):

    content = forms.CharField(widget=forms.Textarea(attrs={'placeholder': 'Hej, jeśli masz problem z aplikacją lub masz pomysł jak ją ulepszyć, daj mi znać!'}), label='', )

class UserEditForm(forms.ModelForm):
    class Meta:
        model = User
        fields = ('first_name', 'last_name', 'email')

class UserSettingsForm(forms.ModelForm):
    class Meta:
        model = UserSettings
        fields = ['site_name', 'site_url', 'phone_number']
        labels = {
            'site_name': 'Nazwa strony',
            'site_url': 'Adres strony',
            'phone_number': 'Telefon',
        }

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