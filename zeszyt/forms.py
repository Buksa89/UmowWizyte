from django import forms

class LoginForm(forms.Form):
    username = forms.CharField(label='Imię')
    password = forms.CharField(widget=forms.PasswordInput, label='Hasło')