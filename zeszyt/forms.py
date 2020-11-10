from django import forms
from django.core.exceptions import NON_FIELD_ERRORS
from .models import Client

class LoginForm(forms.Form):
    username = forms.CharField(label='Imię', error_messages={'required': 'Podaj login'})
    password = forms.CharField(widget=forms.PasswordInput, label='Hasło',
                               error_messages={'required': 'Podaj hasło'})


class ClientLoginForm(forms.Form):
    username = forms.CharField(label='Telefon', error_messages={'required': 'Podaj numer telefonu'})
    password = forms.CharField(label='pin', error_messages={'required': 'Podaj pin'})


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