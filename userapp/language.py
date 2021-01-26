import random

def get_one(text):
    return random.choice(text)

#Registration:
FIELD_REQUIRED = ['To pole nie może być puste']
FIELD_TOO_LONG = ["Za długie to",]
FIELD_WRONG = ["Sprawdź to jeszcze raz"]
YOUR_USERNAME = 'Twoja nazwa użytkownika'
YOUR_FIRST_NAME = 'Jak masz na imię?'
YOUR_LAST_NAME =  'I na nazwisko?'
YOUR_EMAIL = 'Podaj jeszcze adres email'
YOUR_PASSWORD = 'I hasło'
REPEAT_PASSWORD = 'Dla pewności powtórz hasło'
PASSWORDS_DIFFERENT = 'Hasła się różnią'
USERNAME_ALREADY_EXIST = 'Nazwa zajęta'
EMAIL_ALREADY_EXIST = 'Email zajęty'
PASSWORD_TOO_WEAK = 'Hasło powinno mieć co najmniej siedem znaków. W tym literkę małą, dużą, cyfrę i jakiś znak specjalny'
WELCOME_ABOARD = 'Witaj na pokładzie!<br />Możesz się teraz zalogować'
REGISTRATION_TITLE = "Zakładamy nowe konto"
RETURN_TO_MAIN_PAGE = "Powrót do strony głównej"
LOGIN_PAGE = "Logowanie"
READY_BUTTON = "Gotowe"