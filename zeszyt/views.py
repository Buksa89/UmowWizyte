from django.http import HttpResponse
from django.contrib.auth import authenticate, login
from django.contrib.auth.models import User
from django.contrib.auth.decorators import login_required
from django.views.decorators.http import require_http_methods
from django.db import transaction
from django.db.utils import IntegrityError
from django.shortcuts import get_object_or_404, redirect, render
from unittest import skip
from .forms import LoginForm, AddClientForm, ClientLoginForm, AddServiceForm, ClientChooseVisitForm
from .models import Client, Service
from random import choice
from datetime import datetime
from django.urls import reverse
from calendar import HTMLCalendar
from django.utils.safestring import mark_safe

def welcome_screen(request):
    return render(request, 'welcome.html', {})

def login_screen(request):
    if request.user.is_authenticated:
        return redirect(panel_screen)
    else:
        if request.method == 'POST':
            form = LoginForm(request.POST)

            if form.is_valid():
                cd = form.cleaned_data
                user = authenticate(username=cd['username'],
                                    password=cd['password'])
                if user:
                    login(request, user)
                    return redirect(panel_screen)
                elif User.objects.filter(username=cd['username'], is_active=False):
                    form.clean()
                    form.add_error(None, 'Konto zablokowane')
                else:
                    form.clean()
                    form.add_error(None, 'Błędny login lub hasło')
        else:
            form = LoginForm()

        return render(request, 'login.html', {'form':form})

@login_required
def clients_screen(request):
    user = User.objects.get(username=request.user)
    clients = Client.objects.filter(user=user)
    return render(request, 'dashboard/clients.html', {"clients":clients,
                                                  'section':'clients'})


@login_required
def add_client_screen(request):
    # TODO: Refaktoryzacja, przeniesienie do clients
    if request.method == 'POST':
        request.POST._mutable = True
        request.POST['pin'] = pin_generate()
        form = AddClientForm(data=request.POST)
        if form.is_valid():
            user = User.objects.get(username=request.user)
            try:
                form.save(user=user)
                form = AddClientForm()
                return render(request, 'dashboard/add_client.html', {'form': form, 'created_name':request.POST.get('name','')})
            except IntegrityError:
                # TODO: Tą walidację przenieś do formularza
                form.add_error(None, 'Klient o podanym numerze telefonu już istnieje')
                return render(request, 'dashboard/add_client.html', {'form': form})
    else:
        form = AddClientForm()
    return render(request, 'dashboard/add_client.html', {'form': form})

@login_required
def remove_client_screen(request, client_id):
    #TODO: Dodaj potwierdzenie usunięcia
    user = User.objects.get(username=request.user)
    Client.objects.filter(id=client_id,user=user).delete()
    return redirect(clients_screen)

@login_required
def panel_screen(request):
    return render(request, 'dashboard/panel.html', {'section':'panel'})

@login_required
def settings_screen(request):
    service_form = AddServiceForm()
    user = User.objects.get(username=request.user)
    services = Service.objects.filter(user=user)
    created_name = ''   # jeśli zostanie utworzona nowa usługa, wyświetli się powiadomienie z jej nazwą
    if request.method == 'POST':
        service_form = AddServiceForm(data=request.POST)
        if service_form.is_valid():
            if request.POST['duration']=='00:00':   # Usługa nie może trwać 0
                service_form.add_error(None, 'Ustaw czas')
            else:
                try:
                    with transaction.atomic():          # Bez tego wyrzuca błąd
                        service_form.save(user=user)
                    created_name = request.POST.get('name', '')
                    service_form = AddServiceForm()
                except IntegrityError:
                    # TODO: Tą walidację przenieś do formularza
                    service_form.add_error(None, 'Usługa o tej nazwie już istnieje')
    return render(request, 'dashboard/settings.html', {'service_form': service_form,
                                                   'services': services,
                                                   'created_name': created_name,
                                                   'section':'settings'})

@login_required
def remove_service(request,service_id):
    #TODO: Dodaj potwierdzenie usunięcia
    user = User.objects.get(username=request.user)
    #TODO get?
    Service.objects.filter(id=service_id,user=user).delete()
    return redirect(settings_screen)

def client_app(request, username):
    user = get_object_or_404(User, username__iexact=username)
    if is_client_authenticated(request, user.username):
        client = Client.objects.get(phone_number=request.session['client_authorized']['phone'], user=user)
        return client_dashboard(request, user, client)
    else:
        return client_login(request, user)

@require_http_methods(["POST"])
def client_app_new_visit_step_1(request, username):
    service = request.POST.get('service', '')
    return render(request, 'client_app/client_app_new_visit_step_1.html', {'username':username, 'service':service})

def client_dashboard(request, user, client):
    form = ClientChooseVisitForm(user)
    return render(request, 'client_app/client_dashboard.html', {'form':form,
                                                                'username':user.username,
                                                                'client_name':client.name,
                                                                'section':'dashboard'})


def client_login(request, user):
    if request.method == 'POST':
        form = ClientLoginForm(data=request.POST)
        if form.is_valid():
            cd = form.cleaned_data
            client = Client.objects.filter(phone_number=cd['phone_number'],user=user)
            if not client: form.add_error(None, f'Nie ma takiego numeru w bazie {user.username}')
            elif client[0].pin != cd['pin']: form.add_error(None, 'Dane nieprawidłowe')
            elif not client[0].is_active:
                #TODO: Przetestuj czy ta funkcja działa, kiedy już będzie możliwość blokowania klienta
                form.clean()
                form.add_error(None, 'Konto zablokowane')
            else:
                request.session['client_authorized'] = {'phone': cd['phone_number'], 'user':user.username}
                return client_app(request, user)
    else:
        form = ClientLoginForm()

    return render(request, 'client_app/client_login.html', {'form':form, 'user':user.username})

def client_logout(request, username):
    # TODO: testy tej funkcji
    request.session.pop('client_authorized', None)
    return redirect(client_app, username)


# Funkcje pomocnicze
def pin_generate():
    """ Funkcja generuje losowy, 4-cyfrowy pin """
    pin = ''
    for i in range(0,4): pin += choice('0123456789')
    return(pin)

def is_client_authenticated(request, username):
    if request.session.get('client_authorized') and request.session.get('client_authorized')['user'] == username:
        return True
    else: return False

@login_required
def schedule_screen(request, year=datetime.now().year, month=datetime.now().month, day=False):
    visits={}
    if day:
        return render(request, 'dashboard/schedule.html', {'section': 'schedule'})
    calendar = Calendar(year, month, visits).formatmonth()
    return render(request, 'dashboard/calendar.html', {'section':'schedule', 'calendar':mark_safe(calendar)})


def get_month_name(month):
    months = ['Styczeń', 'Luty', 'Marzec', 'Kwiecień', 'Maj', 'Czerwiec', 'Lipiec', 'Sierpień', 'Wrzesień', 'Październik',
          'Listopad', 'Grudzień']
    return(months[month-1])

def get_day_name(day):
    days = ['Pn', 'Wt', 'Śr', 'Cz', 'Pt', 'So', 'Nd']
    return(days[day-1])

class Calendar(HTMLCalendar):
    def __init__(self, year=None, month=None, visits=None):
      self.year = year
      self.month = month
      self.visits = visits
      super(Calendar, self).__init__()

    def formatmonthname(self, theyear, themonth):

        s = f'<li>{get_month_name(themonth)}<br /><span class="year">{theyear}</span></li>'
        return s

    def formatday(self, day):
        #Iteracja wizyt
        if day != 0:
            day_li = f'<a href="{self.get_day_url(day)}"><li>'
            if datetime.today().date() == datetime(self.year, self.month, day).date():
                day_li += f'<span class="active">{day}</span>'
            else: day_li += str(day)
            day_li += '</li></a>'
            return day_li
        return '<li></li>'

    def formatweekheader(self):
        s = ''.join(self.formatweekday(i) for i in self.iterweekdays())
        s = ''.join(f'<li>{get_day_name(i+1)}</li>' for i in self.iterweekdays())
        return s

    def formatweek(self, theweek):
        week = ''
        for d, weekday in theweek:
            week += self.formatday(d)
        return f'<ul class="days"> {week} </ul>'

    def formatmonth(self):
        events={}
        cal = f'<div class="month"><ul><a href="{self.get_month_url("prev")}"><li class="prev">&#10094;</li></a>' \
              f'<a href="{self.get_month_url("next")}"><li class="next">&#10095;</li></a>' \
              f'{self.formatmonthname(self.year, self.month)}\n</ul></div>'
        cal += f'<ul class="weekdays">{self.formatweekheader()}\n</ul>'
        for week in self.monthdays2calendar(self.year, self.month):
            cal += f'{self.formatweek(week)}\n</ul>'
        return cal

    def get_month_url(self, direction):
        month = self.month
        year = self.year
        if direction == "next":
            if month != 12: month += 1
            else:
                year += 1
                month = 1
        if direction == 'prev':
            if month != 1: month -= 1
            else:
                year -= 1
                month = 12
        return reverse('schedule_screen', args=[year, month])

    def get_day_url(self, day):
        return reverse('schedule_screen', args=[self.year, self.month, day])