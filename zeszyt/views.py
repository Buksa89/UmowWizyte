from calendar import HTMLCalendar
from datetime import date, datetime, timedelta
from django.contrib.auth import authenticate, login
from django.contrib.auth.decorators import login_required
from django.utils.decorators import method_decorator
from django.contrib.auth.models import User
from django.db import transaction
from django.db.models import Q
from django.db.utils import IntegrityError
from django.shortcuts import get_object_or_404, redirect, render
from django.urls import reverse
from django.utils import timezone
from django.utils.decorators import method_decorator
from django.utils.safestring import mark_safe
from django.views import View
from django.views.generic.edit import CreateView
from functools import wraps
import holidays
from .forms import AddClientForm, AddServiceForm, AddVisit, ClientChooseVisitForm, ClientLoginForm, LoginForm, WorkTimeForm
from .models import Client, Service, Visit, WorkTime

DAYS_OF_WEEK = ['Poniedziałek', 'Wtorek', 'Środa', 'Czwartek', 'Piątek', 'Sobota', 'Niedziela']



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
    get_object_or_404(Client, id=client_id,user=user).delete()
    return redirect(dashboard_clients)


class DashboardSchedule(View):
    template_schedule_name = 'dashboard/schedule.html'
    template_calendar_name = 'dashboard/schedule_calendar.html'
    section = 'dashboard_schedule'

    @method_decorator(login_required)
    def get(self, request, year=datetime.now().year, month=datetime.now().month, day=False):
        work_time = get_object_or_404(WorkTime, user=request.user)
        if day:
            schedule = ScheduleCalendar(year, month, day ,work_time=work_time)
            return render(request,self.template_schedule_name, {'section':self.section, 'schedule':schedule.display})

        calendar = ScheduleCalendar(year, month, work_time=work_time).formatmonth()

        return render(request, self.template_calendar_name, {'section':self.section, 'calendar':'tu bedzie terminarz'})


class DashboardSettings(CreateView):
    template_name = 'dashboard/settings.html'



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
        work_time_changed = False
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
            work_time_form, work_time_changed = self.dashboard_settings_work_time(request, user)

        return render(request, 'dashboard/settings.html', {'work_time_form': work_time_form,
                                                        'service_form': service_form,
                                                       'services': services,
                                                       'created_service': created_service,
                                                       'work_time_changed': work_time_changed,
                                                       'section':'dashboard_settings'})

    def dashboard_settings_services(self, request, user):
        service_form = AddServiceForm(data=request.POST)
        created_service = None
        if service_form.is_valid():
            if request.POST['duration'] == '00:00:00':  # Usługa nie może trwać 0
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
            return form, True
        return form, False


@login_required
def dashboard_settings_service_remove(request,service_id):
    #TODO: Dodaj potwierdzenie usunięcia
    user = User.objects.get(username=request.user)
    get_object_or_404(Service, id=service_id,user=user).delete()
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

def client_login_required(function):
    @wraps(function)
    def wrap(request, *args, username, **kwargs):
        if is_client_authenticated(request, username):
            #TODO: username w sesji zgadza sie z username strony
            return function(request, *args, username, **kwargs)
        else:
            return redirect('client_app_login', username)
    return wrap


class ClientAppLogin(CreateView):
    template = 'client_app/login.html'
    template_banned = 'client_app/login_not_active.html'

    def get(self, request, username):
        if is_client_authenticated(request, username): return redirect('client_app_dashboard', username)
        user = get_object_or_404(User, username__iexact=username)
        if user.is_active:
            form = ClientLoginForm()
            return render(request, self.template, {'form':form, 'user':user.username})
        else:
            return render(request, self.template_banned, {'user': user.username})

    def post(self, request, username):
        if is_client_authenticated(request, username): return redirect('client_app_dashboard', username)
        user = get_object_or_404(User, username__iexact=username)
        form = ClientLoginForm(data=request.POST)
        if form.is_valid():
            cd = form.cleaned_data
            client = Client.objects.filter(phone_number=cd['phone_number'], user=user)
            if not client:
                form.add_error(None, f'Nie ma takiego numeru w bazie {user.username}')
            elif client[0].pin != cd['pin']:
                form.add_error(None, 'Dane nieprawidłowe')
            elif not client[0].is_active:
                # TODO: Przetestuj czy ta funkcja działa, kiedy już będzie możliwość blokowania klienta
                form.clean()
                form.add_error(None, 'Konto zablokowane')
            else:
                # TODO: SPRAWDZ CZY USER JEST AKTYWNY
                request.session['client_authorized'] = {'phone': cd['phone_number'], 'user': user.username}
                return redirect('client_app_dashboard', username)
        return render(request, self.template, {'form':form, 'user':user.username})



class ClientAppDashboard(View):
    template = 'client_app/dashboard.html'

    @method_decorator(client_login_required)
    def get(self, request, username):
        user = get_object_or_404(User, username__iexact=username)
        client = get_object_or_404(Client, phone_number=request.session.get('client_authorized')['phone'],user=user)
        visits_query = Visit.objects.filter(user=user, client=client)
        visits = []
        for visit in visits_query:
            dict = {}
            dict['name'] = visit.name
            dict['date'] = visit.start.strftime("%Y-%m-%d")
            dict['day'] = DAYS_OF_WEEK[visit.start.weekday()]
            dict['time'] = visit.start.strftime("%H:%M")
            dict['duration'] = str(visit.stop - visit.start)[:-3].rjust(5,'0')
            dict['is_confirmed'] = visit.is_confirmed
            dict['id'] = visit.id
            visits.append(dict)

        form = ClientChooseVisitForm(user)
        return render(request, self.template, {'form':form,
                                               'username':user.username,
                                               'client_name':client.name,
                                               'visits':visits,
                                               'section':'dashboard'})

    @method_decorator(client_login_required)
    def post(self, request, username):
        if 'service' in request.POST.keys(): return redirect('client_app_new_visit', username, request.POST['service'])
        return redirect('client_app_dashboard', username)




#TODO: !!! Testy widoku
class ClientAppNewVisit(View):
    template_name = ''
    section = ''
    def get(self, request, username, service_id, year=datetime.now().year, week = datetime.now().isocalendar()[1]):
        user = get_object_or_404(User, username__iexact=username)
        client = get_object_or_404(Client, phone_number=request.session.get('client_authorized')['phone'],user=user)
        service = get_object_or_404(Service, id=service_id, user=user, is_active=True)
        work_time = get_object_or_404(WorkTime, user=user)
        available_dates = get_available_days_for_clients(username)
        schedule = ClientSchedule(year, week, work_time, service_id, service.name, username, available_dates, client)
        return render(request, 'client_app/new_visit.html',
                      {'section': 'schedule', 'service': service.name,
                       'schedule': schedule.display_week, 'username': username})


#TODO: !!! Testy widoku
class ClientAppConfirmVisit(View):
    template_name = ''
    section = ''
    form = AddVisit()


    def get(self, request, username, service_id, year, month, day, hour, minute):
        #TODO: Walidacja czy na pewno data i godzina wolne
        user = get_object_or_404(User, username__iexact=username)
        service = get_object_or_404(Service, id=service_id, user=user)
        day_number = datetime(year, month, day).weekday()
        day_name = DAYS_OF_WEEK[day_number]
        day_str = str(day).rjust(2,'0')
        month_str = str(month).rjust(2,'0')
        hour_str = str(hour).rjust(2,'0')
        minute_str = str(minute).rjust(2,'0')
        date = f'{day_str}-{month_str}-{year}'
        time = f'{hour_str}:{minute_str}'

        return render(request, 'client_app/confirm_visit.html', {'section': '', 'service': service.name, 'date': date,
                                                                 'day_name': day_name, 'time': time, 'form':self.form,
                                                                 'username': username, 'service_id': service_id,
                                                                 'year':year, 'month':month, 'day':day, 'hour':hour,
                                                                 'minute':minute})

    def post(self, request, username, service_id, year, month, day, hour, minute):
        #TODO: Walidacja czy na pewno data i godzina wolne
        # jeśli tak, redirect do strony glownej
        # jesli nie, render jeszcze raz
        # Potwierdzenie na głównej

        form = AddVisit(data=self.request.POST)
        user = get_object_or_404(User, username__iexact=username)
        client = get_object_or_404(Client, phone_number=request.session.get('client_authorized')['phone'])
        service = get_object_or_404(Service, id=service_id, user=user)
        name = service.name
        start = datetime(year, month, day, hour, minute)
        stop = start + service.duration
        is_available = True
        is_confirmed = False

        form.save(user, client, name, start, stop, is_available, is_confirmed)

        return redirect('client_app_dashboard', username)


class ClientAppCancelVisit(View):

    def get(self, request, username, visit_id):
        user = get_object_or_404(User, username__iexact=username)
        client = get_object_or_404(Client, phone_number=request.session['client_authorized']['phone'], user=user)
        Visit.objects.filter(user=user, client=client, id=visit_id).delete()

        return redirect('client_app_dashboard', username)



class ClientAppLogout(View):
    def get(self, request, username):
        request.session.pop('client_authorized', None)
        return redirect('client_app_login', username)


""" Other Views """

def welcome_screen(request):
    users = User.objects.filter(~Q(username='admin'))
    return render(request, 'welcome.html', {'users':users})


""" Funkcje pomocnicze """

def is_client_authenticated(request, username):
    """ Check client is logged in. Equivalent of 'user.is_authenticated' """
    if not request.session.get('client_authorized'): return False
    if request.session.get('client_authorized')['user'].lower() != username.lower(): return False
    return True


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
        available_date.append(date.date())
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


class ClientCalendar(Calendar):
    """ Class generate calendar for choose visit by clients """

    def __init__(self, username, service_id, service_name, visits, year, month):
        self.service_name = service_name
        self.username = username
        self.service_id = service_id
        self.visits = visits
        self.year = year
        self.month = month
        super(Calendar, self).__init__()

    def formatmonthname(self, theyear, themonth):

        s = f'<li>{self.service_name}<br /><span class="year">{self.get_month_name(themonth)} {theyear}</span></li>'
        return s

    def formatday(self, day):
        if day != 0:
            span_class = ''
            li_class = ''
            href_open = ''
            href_close = ''
            if is_holiday(datetime(self.year, self.month, day).date()): span_class = ' class="red"'
            elif datetime.today().date() == datetime(self.year, self.month, day).date(): span_class = ' class="active"'
            is_available_day = self.is_available_day(datetime(self.year, self.month, day).date(), self.visits)
            if is_available_day:
                href_open = f'<a href="{self.get_day_url(day)}">'
                href_close = '</a>'
                li_class = ' class="available"'
            if not is_available_day: li_class = ' class="dark"'
            day_li = f'{href_open}<li{li_class}>'
            day_li += f'<span{span_class}>{day}</span>'
            day_li += f'</li>{href_close}'
            return day_li
        return '<li></li>'

    def formatmonth(self):

        cal = f'<div class="month"><ul>'
        #TODO: Dodaj ograniczenie kalendarza, kiedy już dalej nie ma wolnych terminów
        #TODO: Zamiast wczytywać wszystkie wolne terminy, można wczytywać tylko te z wyświetlonego miesiąca
        if True:
            cal += f'<a href="{self.get_month_url("prev")}"><li class="prev">&#10094;</li></a>'
        if True:
            cal += f'<a href="{self.get_month_url("next")}"><li class="next">&#10095;</li></a>'
        cal += f'{self.formatmonthname(self.year, self.month)}\n</ul></div>'
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
        return reverse('client_app_new_visit_1', args=[self.username, self.service_id, year, month])


    def is_available_day(self, date, visit):
        if date in visit: return True
        else: return False

    def get_day_url(self, day):
        """ Method return link for choose visit by clients """
        return reverse('client_app_new_visit_1', args=[self.username, self.service_id, self.year, self.month, day])


class ClientSchedule:

    def __init__(self, year, week, work_time, service_id, service_name, username, available_dates, client):
        self.client = client
        self.start_work_time = work_time.start_time
        self.end_work_time = work_time.end_time
        self.work_time = work_time
        self.service_id = service_id
        self.user = get_object_or_404(User, username__iexact=username)
        self.username = username
        self.available_dates = available_dates
        self.year = year
        self.week = week
        self.service_name = service_name
        self.day = self.get_dates_from_week(year, week)
        self.DAYS_OF_WEEK = ['Pn','Wt','Śr','Cz','Pt','So','Nd']

    def display_header(self):

        # Prepare links to navigate

        prev_week_url = self.get_week_url(
            (self.day[0] - timedelta(days=7)).year,
            self.get_week_from_date(self.day[0] - timedelta(days=7)))

        next_week_url = self.get_week_url(
            (self.day[0] + timedelta(days=7)).year,
            self.get_week_from_date(self.day[0] + timedelta(days=7)))

        html_code = f'<div class="header"><ul>' \
                    f'<a href="{prev_week_url}"><li class="prev">&#10094;</li></a>' \
                    f'<a href="{next_week_url}"><li class="next">&#10095;</li></a>' \
                    f'<li>{self.service_name}</li>' \
                    f'</ul></div>'

        return html_code

    def display_dates_header(self):
        html_code = '<div class="dates-header">'
        html_code += '<ul><li class="empty"></li>'

        # Years and months
        years_and_months_to_display_dict = self.get_years_and_months(self.day)
        for n, data in enumerate(years_and_months_to_display_dict):
            css_class = ''
            if len(years_and_months_to_display_dict) > 1 and n == len(years_and_months_to_display_dict)-1:
                css_class = 'border-date'
            html_code += f"<li style=\"width:{12.5*data['count']}%\" class=\"{css_class}\">" \
                         f"{data['year']}<br />" \
                         f"{self.get_month_name(data['month'])}" \
                         f"</li>"
        html_code += '</ul>'

        # Days

        html_code += f'<ul><li class="empty"></li>'
        for n, day in enumerate(self.day):
            css_class = []
            if is_holiday(day) or n == 6: css_class.append('red')
            if datetime.today().date() == day.date(): css_class.append('today')
            if n == years_and_months_to_display_dict[0]['count']: css_class.append('border-date')
            css_class = ' '.join(css_class)
            html_code += f'<li class="{css_class}"><div>{day.day}' \
                         f'<br />{self.DAYS_OF_WEEK[n]}</div></li>'
        html_code += f'</ul></div>'

        return html_code

    def display_week(self):

        html_code = self.display_header()
        html_code += self.display_dates_header()

        html_code += f'<ul class="schedule-content"><li><ul class="hours">'
        # Hours column
        hour = self.start_work_time
        work_hours = []
        while hour < self.end_work_time:
            work_hours.append(hour)
            html_code += f'<li>{hour.strftime("%H:%M")}</li>'
            hour = (datetime.combine(date.today(), hour) + timedelta(minutes=15)).time()
        html_code += '</ul></li>'

        for day in self.day:
            if day.date() not in self.available_dates:
                html_code += '<li><ul>'
                for hour in work_hours:
                    html_code += f'<li>{hour.strftime("%H:%M")}</li>'
                html_code += '</ul></li>'
            else:
                html_code += '<li><ul>'

                #
                for hour in work_hours:
                    full_term = datetime.combine(day, hour)

                    html_code += f'<a href="{self.get_term_url(full_term)}"><li>{hour.strftime("%H:%M")}</li></a>'
                html_code += '</ul></li>'
            print(self.get_available_hours_in_day(day.date()))
        html_code += f'</ul>'
        #
        # Pobranie godzin

        """for n, hour in enumerate(work_hours):
                    full_term = datetime.combine(day, hour)
                    html_code += f'<li class="avaliable">{hour.strftime("%H:%M")}</li>'
                html_code += '</ul></li>'
        html_code += f'</ul>'"""



        return html_code

    def get_years_and_months(self, dates):
        var = []
        for date_ in dates:
            if not var:
                var.append({'month': date_.month, 'year': date_.year, 'count': 1})
            else:
                exist = False
                for n, data in enumerate(var):
                    if var[n]['month'] == date_.month and var[n]['year'] == date_.year:
                        var[n]['count'] += 1
                        exist = True
                        break
                if not exist:
                    var.append({'month': date_.month, 'year': date_.year, 'count': 1})
        return var

    def get_week_url(self, year, week):
        return reverse('client_app_new_visit', args=[self.username, self.service_id, year, week])

    def get_week_from_date(self, date):
        return date.isocalendar()[1]

    def get_dates_from_week(self, year, week):
        days = []
        days.append(datetime.fromisocalendar(year, week, 1))
        for i in range(0,6):
            days.append(days[-1] + timedelta(days=1))
        return days

    def get_month_name(self, month):
        """ Method  return polish name of month """
        months = ['Styczeń', 'Luty', 'Marzec', 'Kwiecień', 'Maj', 'Czerwiec', 'Lipiec', 'Sierpień', 'Wrzesień',
                  'Październik', 'Listopad', 'Grudzień']
        return (months[month - 1])

    def get_term_url(self, term):
        """ Method return link for choose visit by clients """
        return reverse('client_app_confirm_visit', args=[self.username, self.service_id,
                                                         term.year, term.month, term.day, term.hour, term.minute])

    def get_available_hours_in_day(self, date):
        visits = Visit.objects.filter(Q(user=self.user, client=self.client, start__year=date.year, start__month=date.month, start__day=date.day) |
                                      Q(user=self.user, client=self.client, stop__year=date.year, stop__month=date.month, stop__day=date.day))

        #'[[self.work_time.start_time.strftime("%H:%M"),self.work_time.end_time.strftime("%H:%M")]]

        start_work = timezone.make_aware(datetime.combine(date,self.work_time.start_time), timezone.get_default_timezone())
        end_work = timezone.make_aware(datetime.combine(date,self.work_time.end_time), timezone.get_default_timezone())
        time_zero = timezone.make_aware(datetime(date.year, date.month, date.day, 0, 0), timezone.get_default_timezone())
        time_midnight = timezone.make_aware(datetime(date.year, date.month, date.day, 23, 45), timezone.get_default_timezone())
        avaliable_hours = [[time_zero, start_work], [time_midnight, end_work]]

        for visit in visits:
            #TODO Zmien w calym projekcie stop na end
            if visit.start <= time_zero: visit.start = time_zero
            if visit.stop >= time_midnight: visit.stop = time_midnight
            avaliable_hours.append([visit.start, visit.stop])

        def sort_by_start_time(period):
            return period[0]

        avaliable_hours.sort(key=sort_by_start_time)


                #if visit.stop < start_av_period:
                    #avaliable_hours.insert(n, "new")

                #print(visit.start)
                #print(visit.stop)


        for n, periods in enumerate(avaliable_hours):
            for m, date in enumerate(periods):
                avaliable_hours[n][m] = date.strftime('%H:%M')
        return avaliable_hours