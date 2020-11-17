from django import forms
from django.core.exceptions import NON_FIELD_ERRORS, ValidationError
from .models import Client, Service, WorkTime


def time_choices(hours=8):
    """ Generating list of times from 0 to hours. Step = 15min"""
    choices = []
    for hour in range(0, hours):
        hour = str(hour).rjust(2, '0')
        hour += ':'
        for minute in range(00,59,15):
            minute = str(minute).rjust(2, '0')
            choices.append([hour+minute,hour+minute])

    last = str(hours).rjust(2, '0') + ":00"
    choices.append([last,last])
    choices=tuple(choices)
    return choices


class LoginForm(forms.Form):
    username = forms.CharField(label='Imię', error_messages={'required': 'Podaj login'})
    password = forms.CharField(widget=forms.PasswordInput, label='Hasło',
                               error_messages={'required': 'Podaj hasło'})


class ClientLoginForm(forms.Form):
    phone_number = forms.CharField(label='Telefon', error_messages={'required': 'Podaj numer telefonu'})
    pin = forms.CharField(label='pin', error_messages={'required': 'Podaj pin'})


class AddClientForm(forms.ModelForm):

    def save(self, user):
        self.instance.user = user
        return super().save()

    def only_int(value):
        if value.isdigit() == False:
            raise ValidationError('Podaj prawidłowy numer telefonu')

    phone_number = forms.CharField(validators=[only_int],
                                   error_messages={'required': 'Pole nie może być puste',
                                                   'max_length': 'Numer jest za długi'},
                                   widget=forms.TextInput(attrs={'placeholder': 'Telefon'}),
                                   label='Telefon*',)

    class Meta:
        model = Client
        fields = ['phone_number', 'email', 'name', 'surname', 'description','pin']
        labels = {
            'phone_number': 'Telefon*',
            'email': 'email',
            'name': 'Imię*',
            'surname': 'Nazwisko',
            'description': 'Dodatkowe info',
            'pin': ''
        }
        widgets = {
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
            'name': {'required': "Pole nie może być puste",
                     'max_length': 'Nazwa jest za długa'},
            'email': {'invalid': 'Email nieprawidłowy','max_length': 'Email jest za długi'},
            'surname': {'max_length': 'Nazwisko jest za długie'},
            'description': {'max_length': 'Opis jest za długi'},
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
            'duration': forms.Select(choices=time_choices()),
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


class WorkTimeForm(forms.ModelForm):
    # TODO: Walidacja czy czas zakonczenia nie jest wczesniej niz rozpoczecia
    # TODO: Wczytanie danych do formularza
    # TODO: JEśli nie ma danych, utwórz prrzykładowe
    # TODO: - zapis na TYM SAMYM miejscu
    class Meta:
        model = WorkTime
        fields = ['start_time', 'stop_time', 'monday', 'tuesday', 'wednesday',
                  'thursday', 'friday', 'saturday', 'sunday', 'holidays']
        labels = {
            'start_time': 'Od godziny',
            'stop_time': 'Do godziny',
            'monday': 'Poniedziałki',
            'tuesday': 'Wtorki',
            'wednesday': 'Środy',
            'thursday': 'Czwartki',
            'friday': 'Piątki',
            'saturday': 'Soboty',
            'sunday': 'Niedziele',
            'holidays': 'Święta'
        }
        widgets = {
            'start_time': forms.Select(choices=time_choices(24)),
            'stop_time': forms.Select(choices=time_choices(24)),
            'monday': forms.CheckboxInput(),
            'tuesday': forms.CheckboxInput(),
            'wednesday': forms.CheckboxInput(),
            'thursday': forms.CheckboxInput(),
            'friday': forms.CheckboxInput(),
            'saturday': forms.CheckboxInput(),
            'sunday': forms.CheckboxInput(),
            'holidays': forms.CheckboxInput(),
            }
