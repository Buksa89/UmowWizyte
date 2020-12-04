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
from .forms import AddClientForm, AddServiceForm, LoginForm, WorkTimeForm
from .models import Client, Service, Visit, WorkTime




""" User Views """

@login_required
def dashboard(request):
    return render(request, 'dashboard.html', {'section':'dashboard'})


@login_required
def dashboard_clients(request):
    #TODO: wylogowanie jesli nie jest autoryzowany
    user = User.objects.get(username=request.user)
    clients = Client.objects.filter(user=user)
    return render(request, 'clients.html', {"clients":clients,
                                                  'section':'dashboard_clients'})


class DashboardClientsAdd(CreateView):
    template_name = 'clients_add.html'
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
    template_schedule_name = 'schedule.html'
    template_calendar_name = 'schedule_calendar.html'
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
    template_name = 'settings.html'



    @method_decorator(login_required)
    def get(self, request):
        user = User.objects.get(username=request.user)
        services = Service.objects.filter(user=user)
        service_form = AddServiceForm()
        work_time = WorkTime.objects.get(user=user)
        work_time.__dict__['start_time'] = work_time.__dict__['start_time'].strftime("%H:%M")
        work_time.__dict__['end_time'] = work_time.__dict__['end_time'].strftime("%H:%M")
        work_time_form = WorkTimeForm(initial=work_time.__dict__,instance=work_time)

        return render(request, 'settings.html', {'work_time_form': work_time_form,
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

        return render(request, 'settings.html', {'work_time_form': work_time_form,
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


""" Other Views """

def welcome_screen(request):
    users = User.objects.filter(~Q(username='admin'))
    return render(request, 'welcome.html', {'users':users})


""" Funkcje pomocnicze """







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

