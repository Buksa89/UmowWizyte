from django import forms
from django.core.exceptions import NON_FIELD_ERRORS
from .models import Client, Service

class LoginForm(forms.Form):
    username = forms.CharField(label='Imię', error_messages={'required': 'Podaj login'})
    password = forms.CharField(widget=forms.PasswordInput, label='Hasło',
                               error_messages={'required': 'Podaj hasło'})


class ClientLoginForm(forms.Form):
    phone_number = forms.CharField(label='Telefon', error_messages={'required': 'Podaj numer telefonu'})
    pin = forms.CharField(label='pin', error_messages={'required': 'Podaj pin'})


class AddClientForm(forms.ModelForm):
    class Meta:
        model = Client
        fields = ['phone_number', 'email', 'name', 'surname', 'description','pin']
        labels = {
            'phone_number': 'Telefon*',
            'email': 'email',
            'name': 'Imię',
            'surname': 'Nazwisko',
            'description': 'Dodatkowe info',
            'pin': ''
        }
        widgets = {
            'phone_number': forms.fields.TextInput(attrs={
                'placeholder': 'Telefon',}),
            'email': forms.fields.TextInput(attrs={
                'placeholder': 'Mail',}),
            'name': forms.fields.TextInput(attrs={
                'placeholder': 'Imię (to pole wyświetla sie klientowi!)',}),
            'surname': forms.fields.TextInput(attrs={
                'placeholder': 'Nazwisko',}),
            'description': forms.fields.TextInput(attrs={
                'placeholder': 'Opis (Tego pola klient nie widzi',}),
            'pin': forms.fields.HiddenInput(),
            }
        error_messages = {
            'phone_number': {'required': "Pole nie może być puste",
                             'invalid': "Podaj prawidłowy numer telefonu"},

            'name': {'required': "Pole nie może być puste"},
            NON_FIELD_ERRORS: {
                'unique_together': "Posiadasz już klienta z tym numerem telefonu",
            }
        }

    def save(self, user):
        self.instance.user = user
        return super().save()




def duration_choices(hours=8):
    """ Funkcja tworzy krotkę zawierającą listę wszystkich godzin - od 0 do hours co 15 min"""
    choices = []
    for hour in range(0, hours):
        hour = str(hour).rjust(2, '0')
        hour += ':'
        for minute in range(00,59,15):
            minute = str(minute).rjust(2, '0')
            choices.append([hour+minute,hour+minute])

    last = str(hours).rjust(2, '0') + ":00"
    choices.append([last,last])
    choices[0][1] = "Czas"
    choices=tuple(choices)

    return choices

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
            'duration': forms.Select(choices=duration_choices()),
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

