from calendar import HTMLCalendar
from datetime import datetime
from datetime import timedelta
from django.contrib.auth import authenticate, login
from django.contrib.auth.decorators import login_required
from django.utils.decorators import method_decorator
from django.contrib.auth.models import User
from django.db import transaction
from django.db.utils import IntegrityError
from django.shortcuts import get_object_or_404, redirect, render
from django.urls import reverse
from django.utils.decorators import method_decorator
from django.utils.safestring import mark_safe
from django.views import View
from django.views.decorators.http import require_http_methods
from django.views.generic.edit import CreateView
import holidays
from .forms import AddClientForm, AddServiceForm, ClientChooseVisitForm, ClientLoginForm, LoginForm, WorkTimeForm
from .models import Client, Service, WorkTime

""" User Views """

@login_required
def dashboard(request):
    return render(request, 'dashboard/dashboard.html', {'section':'dashboard'})


@login_required
def dashboard_clients(request):
    #TODO: wylogowanie jesli nie jest autoryzowany
    user = User.objects.get(username=request.user)
    clients = Client.objects.filter(user=user)
    return render(request, 'dashboard/clients.html', {"clients":clients,
                                                  'section':'dashboard_clients'})


class DashboardClientsAdd(CreateView):
    template_name = 'dashboard/clients_add.html'
    section = 'dashboard_clients'

    @method_decorator(login_required)
    def get(self, request):
        form = AddClientForm()
        return render(request, self.template_name, {'form': form, 'section':self.section})

    @method_decorator(login_required)
    def post(self, request):
        form = AddClientForm(data=self.request.POST)
        if form.is_valid():
            user = User.objects.get(username=request.user)
            form.save(user)
            created_client_name = request.POST.get('name','')
            form = AddClientForm()
            return render(request, self.template_name, {'form': form,
                                                        'created_client_name':created_client_name,
                                                        'section':self.section})

        form = AddClientForm(data=self.request.POST)
        return render(self.request, self.template_name, {'form': form, 'section': 'dashboard_clients'})


@login_required
def dashboard_clients_remove(request, client_id):
    #TODO: Dodaj potwierdzenie usunięcia
    user = User.objects.get(username=request.user)
    Client.objects.filter(id=client_id,user=user).delete()
    return redirect(dashboard_clients)


class DashboardSchedule(View):
    template_schedule_name = 'dashboard/schedule.html'
    template_calendar_name = 'dashboard/schedule_calendar.html'
    section = 'dashboard_schedule'

    @method_decorator(login_required)
    def get(self, request, year=datetime.now().year, month=datetime.now().month, day=False):
        if day:
            return render(request,self.template_schedule_name, {'section':self.section})

        work_time = get_object_or_404(WorkTime, user=request.user)
        calendar = ScheduleCalendar(year, month, work_time=work_time).formatmonth()

        return render(request, self.template_calendar_name, {'section':self.section, 'calendar':mark_safe(calendar)})


class DashboardSettings(CreateView):
    template_name = 'dashboard/settings.html'


    """    initial = {'size':'L'}
    def get_form(self):
        form = super(DashboardSettings, self).get_form()
        initial_base = self.get_initial() 
        initial_base['menu'] = Menu.objects.get(id=1)
        form.initial = initial_base
        form.fields['name'].widget = forms.widgets.Textarea()
        return form"""


    @method_decorator(login_required)
    def get(self, request):
        user = User.objects.get(username=request.user)
        services = Service.objects.filter(user=user)
        service_form = AddServiceForm()
        work_time = WorkTime.objects.get(user=user)
        work_time.__dict__['start_time'] = work_time.__dict__['start_time'].strftime("%H:%M")
        work_time.__dict__['end_time'] = work_time.__dict__['end_time'].strftime("%H:%M")
        work_time_form = WorkTimeForm(initial=work_time.__dict__,instance=work_time)

        return render(request, 'dashboard/settings.html', {'work_time_form': work_time_form,
                                                        'service_form': service_form,
                                                       'services': services,
                                                       'section':'dashboard_settings'})

    @method_decorator(login_required)
    def post(self, request):
        created_service = None
        user = User.objects.get(username=request.user)
        services = Service.objects.filter(user=user)
        service_form = AddServiceForm()
        work_time = WorkTime.objects.get(user=user)
        work_time.__dict__['start_time'] = work_time.__dict__['start_time'].strftime("%H:%M")
        work_time.__dict__['end_time'] = work_time.__dict__['end_time'].strftime("%H:%M")
        work_time_form = WorkTimeForm(initial=work_time.__dict__,instance=work_time)
        if 'submit' not in request.POST.keys(): None
        elif request.POST['submit'] == 'add_service':
            service_form, created_service = self.dashboard_settings_services(request, user)
        elif request.POST['submit'] == 'set_work_time':
            work_time_form = self.dashboard_settings_work_time(request, user)

        return render(request, 'dashboard/settings.html', {'work_time_form': work_time_form,
                                                        'service_form': service_form,
                                                       'services': services,
                                                       'created_service': created_service,
                                                       'section':'dashboard_settings'})

    def dashboard_settings_services(self, request, user):
        service_form = AddServiceForm(data=request.POST)
        created_service = None
        if service_form.is_valid():
            if request.POST['duration'] == '00:00':  # Usługa nie może trwać 0
                service_form.add_error(None, 'Ustaw czas')
            else:
                try:
                    with transaction.atomic():  # Bez tego wyrzuca błąd
                        service_form.save(user=user)
                    created_service = request.POST.get('name', '')
                    service_form = AddServiceForm()
                except IntegrityError:
                    # TODO: Tą walidację przenieś do formularza
                    service_form.add_error(None, 'Usługa o tej nazwie już istnieje')
        return service_form, created_service


    def dashboard_settings_work_time(self, request, user):
        work_time = WorkTime.objects.get(user=user)
        form = WorkTimeForm(data=request.POST, instance=work_time)
        if form.is_valid():
            form.save()
            #TODO: Dodaj potwierdzenie zmiany
        return form


@login_required
def dashboard_settings_service_remove(request,service_id):
    #TODO: Dodaj potwierdzenie usunięcia
    user = User.objects.get(username=request.user)
    Service.objects.get(id=service_id,user=user).delete()
    return redirect('dashboard_settings')


def login_screen(request):
    if request.user.is_authenticated:
        return redirect(dashboard)
    else:
        if request.method == 'POST':
            form = LoginForm(request.POST)

            if form.is_valid():
                cd = form.cleaned_data
                user = authenticate(username=cd['username'],
                                    password=cd['password'])
                if user:
                    login(request, user)
                    return redirect(dashboard)
                elif User.objects.filter(username=cd['username'], is_active=False):
                    form.clean()
                    form.add_error(None, 'Konto zablokowane')
                else:
                    form.clean()
                    form.add_error(None, 'Błędny login lub hasło')
        else:
            form = LoginForm()

        return render(request, 'login.html', {'form':form})


""" Client Views """ # TODO: Do refaktoryzacji

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
    service = get_object_or_404(Service, id=service)
    available_dates = get_available_days_for_clients(username)
    daty = '<ul>'
    for date in available_dates:
        daty += '<li>' + date.strftime("%y-%m-%d") + '</li>'
    daty += '</ul>'



    return render(request, 'client_app/new_visit_calendar.html', {'username':username, 'service':service.name, 'daty':daty})

    #TODO: Tutaj będzie kalendarz









    #print(datetime.today() - timedelta(days=days_to_subtract))

    #print(work_time.latest_visit)



def client_dashboard(request, user, client):
    form = ClientChooseVisitForm(user)
    return render(request, 'client_app/dashboard.html', {'form':form,
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
                #TODO: SPRAWDZ CZY USER JEST AKTYWNY
                request.session['client_authorized'] = {'phone': cd['phone_number'], 'user':user.username}
                return client_app(request, user)
    else:
        form = ClientLoginForm()

    return render(request, 'client_app/login.html', {'form':form, 'user':user.username})

def client_app_logout(request, username):
    request.session.pop('client_authorized', None)
    return redirect(client_app, username)


""" Other Views """

def welcome_screen(request):
    return render(request, 'welcome.html', {})


""" Funkcje pomocnicze """

def is_client_authenticated(request, username):
    """ Check client is logged in. Equivalent of 'user.is_authenticated' """
    if request.session.get('client_authorized') and request.session.get('client_authorized')['user'] == username:
        return True
    else: return False


def get_available_days_for_clients(username):
    """ Function get work_time of user and check what days his clients can choose for visit """
    user = get_object_or_404(User, username__iexact=username)
    work_time = get_object_or_404(WorkTime, user=user)
    available_date = []
    for days in range (work_time.earliest_visit, work_time.latest_visit+1):
        date = datetime.today() + timedelta(days=days)
        if not work_time.holidays and is_holiday(date): continue
        day_name = date.strftime("%A").lower()
        if not work_time.__dict__[day_name]: continue
        available_date.append(date)
    return available_date

def is_holiday(date):
    """ Function get date and return True if it is holiday """
    if date in holidays.Poland(): return True


def is_free_day(date, work_time):
    """ Function get date and return True if it is not working day """
    if not work_time.__dict__[date.strftime("%A").lower()]: return True


class Calendar(HTMLCalendar):
    """ Class generate html calendar """
    def __init__(self, year=None, month=None, work_time=None):
        self.year = year
        self.month = month
        self.work_time = work_time
        super(Calendar, self).__init__()

    def formatmonthname(self, theyear, themonth):
        s = f'<li>{self.get_month_name(themonth)}<br /><span class="year">{theyear}</span></li>'
        return s

    def formatday(self, day):
        if day != 0:
            span_class = ''
            li_class = ''
            if is_holiday(datetime(self.year, self.month, day).date()): span_class = ' class="red"'
            elif datetime.today().date() == datetime(self.year, self.month, day).date(): span_class = ' class="active"'
            if is_free_day(datetime(self.year, self.month, day), self.work_time): li_class = ' class="dark"'
            day_li = f'<li{li_class}>'
            day_li += f'<span{span_class}>{day}</span>'
            day_li += '</li>'
            return day_li
        return '<li></li>'

    def formatweekheader(self):
        s = ''.join(self.formatweekday(i) for i in self.iterweekdays())
        s = ''.join(f'<li>{self.get_day_name(i+1)}</li>' for i in self.iterweekdays())
        return s

    def formatweek(self, theweek):
        week = ''
        for d, weekday in theweek:
            week += self.formatday(d)
        return f'<ul class="days"> {week} </ul>'

    def formatmonth(self):
        events={}
        cal = f'<div class="month"><ul>{self.formatmonthname(self.year, self.month)}\n</ul></div>'
        cal += f'<ul class="weekdays">{self.formatweekheader()}\n</ul>'
        for week in self.monthdays2calendar(self.year, self.month):
            cal += f'{self.formatweek(week)}\n</ul>'
        return cal

    # Funkcje pomocnicze:

    def get_month_name(self, month):
        """ Method  return polish name of month """
        months = ['Styczeń', 'Luty', 'Marzec', 'Kwiecień', 'Maj', 'Czerwiec', 'Lipiec', 'Sierpień', 'Wrzesień',
                  'Październik', 'Listopad', 'Grudzień']
        return (months[month - 1])

    def get_day_name(self, day):
        """ Method return name day of week from day number"""
        days = ('Pn', 'Wt', 'Śr', 'Cz', 'Pt', 'So', 'Nd')
        return(days[day-1])


class ScheduleCalendar(Calendar):
    """ Class generate calendar for Schedule section """

    def formatday(self, day):
        if day != 0:
            span_class = ''
            li_class = ''
            if is_holiday(datetime(self.year, self.month, day).date()): span_class = ' class="red"'
            elif datetime.today().date() == datetime(self.year, self.month, day).date(): span_class = ' class="active"'
            if is_free_day(datetime(self.year, self.month, day), self.work_time): li_class = ' class="dark"'
            day_li = f'<a href="{self.get_day_url(day)}"><li{li_class}>'
            day_li += f'<span{span_class}>{day}</span>'
            day_li += '</li></a>'
            return day_li
        return '<li></li>'

    def formatmonth(self):
        events={}
        cal = f'<div class="month"><ul><a href="{self.get_month_url("prev")}"><li class="prev">&#10094;</li></a>' \
              f'<a href="{self.get_month_url("next")}"><li class="next">&#10095;</li></a>' \
              f'{self.formatmonthname(self.year, self.month)}\n</ul></div>'
        cal += f'<ul class="weekdays">{self.formatweekheader()}\n</ul>'
        for week in self.monthdays2calendar(self.year, self.month):
            cal += f'{self.formatweek(week)}\n</ul>'
        return cal

    # Funkcje pomocnicze:

    def get_month_url(self, direction):
        """ Method return link for next/previous month calendar """
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
        return reverse('dashboard_schedule', args=[year, month])

    def get_day_url(self, day):
        """ Method return link for date schedule """
        return reverse('dashboard_schedule', args=[self.year, self.month, day])

