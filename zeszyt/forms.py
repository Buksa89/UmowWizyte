from django import forms
from .models import Client

class LoginForm(forms.Form):
    username = forms.CharField(label='Imię', error_messages={'required': 'Podaj login'})
    password = forms.CharField(widget=forms.PasswordInput, label='Hasło',
                               error_messages={'required': 'Podaj hasło'})

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
            'phone_number': {'required': "Pole nie może być puste"}
        }

    def save(self, user, pin):
        self.instance.user = user
        self.pin = pin
        return super().save()