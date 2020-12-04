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
import portion as interval
from .forms import AddVisit, ClientChooseVisitForm, ClientLoginForm
from userapp.models import Client, Service, Visit, WorkTime

DAYS_OF_WEEK = ['Poniedziałek', 'Wtorek', 'Środa', 'Czwartek', 'Piątek', 'Sobota', 'Niedziela']

def client_login_required(function):
    @wraps(function)
    def wrap(request, *args, username, **kwargs):
        if is_client_authenticated(request, username):
            return function(request, *args, username, **kwargs)
        else:
            return redirect('client_app_login', username)
    return wrap


class ClientAppLogin(CreateView):
    template = 'client_login.html'
    template_not_active = 'client_login_not_active.html'

    def get(self, request, username):
        if is_client_authenticated(request, username): return redirect('client_app_dashboard', username)
        user = get_object_or_404(User, username__iexact=username)
        if user.is_active:
            form = ClientLoginForm()
            return render(request, self.template, {'form':form, 'user':user.username})
        else:
            return render(request, self.template_not_active, {'user': user.username})

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
    template = 'client_dashboard.html'

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
            dict['duration'] = str(visit.end - visit.start)[:-3].rjust(5,'0')
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
        return render(request, 'client_new_visit.html',
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

        return render(request, 'client_confirm_visit.html', {'section': '', 'service': service.name, 'date': date,
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
        client = get_object_or_404(Client, phone_number=request.session.get('client_authorized')['phone'], user=user)
        service = get_object_or_404(Service, id=service_id, user=user)
        name = service.name
        start = datetime(year, month, day, hour, minute)
        end = start + service.duration
        is_available = True
        is_confirmed = False

        form.save(user, client, name, start, end, is_available, is_confirmed)

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

class ClientSchedule:

    def __init__(self, year, week, work_time, service_id, service_name, username, available_dates, client):
        self.client = client
        self.start_work_time = work_time.start_time
        self.end_work_time = work_time.end_time
        self.work_time = work_time
        self.user = get_object_or_404(User, username__iexact=username)
        self.service = get_object_or_404(Service, id=service_id, user=self.user)
        self.username = username
        self.available_dates = available_dates
        self.year = year
        self.week = week
        self.day = self.get_dates_from_week(year, week)
        self.DAYS_OF_WEEK = ['Pn','Wt','Śr','Cz','Pt','So','Nd']

    def display_header(self):

        # Prepare links to navigate: next week, previous wekk

        prev_week_url = self.get_week_url(
            (self.day[0] - timedelta(days=7)).year,
            self.get_week_from_date(self.day[0] - timedelta(days=7)))

        next_week_url = self.get_week_url(
            (self.day[0] + timedelta(days=7)).year,
            self.get_week_from_date(self.day[0] + timedelta(days=7)))

        html_code = f'<div class="header"><ul>' \
                    f'<a href="{prev_week_url}"><li class="prev">&#10094;</li></a>' \
                    f'<a href="{next_week_url}"><li class="next">&#10095;</li></a>' \
                    f'<li>{self.service.name}</li>' \
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
        while hour <= self.end_work_time:
            work_hours.append(hour)
            html_code += f'<li>{hour.strftime("%H:%M")}</li>'
            hour = (datetime.combine(date.today(), hour) + timedelta(minutes=15)).time()
        html_code += '</ul></li>'

        simple_duration = int(self.service.duration / timedelta(minutes=15))
        print(simple_duration)

        for day in self.day:
            if day.date() not in self.available_dates:
                html_code += '<li><ul>'
                for hour in work_hours:
                    html_code += f'<li>{hour.strftime("%H:%M")}</li>'
                html_code += '</ul></li>'
            else:
                html_code += '<li><ul>'

                av_time = self.get_available_hours_in_day(day.date())
                for i, hour in enumerate(work_hours):
                    li_class = ""
                    if not av_time[hour.strftime("%H:%M")]:
                        html_code += f'<li>{hour.strftime("%H:%M")}</li>'
                    else:
                        full_term = datetime.combine(day, hour)
                        flag = True
                        for n in range(simple_duration):
                            try:
                                if av_time[work_hours[i+n].strftime("%H:%M")]:
                                    continue
                            except: pass
                            flag = False
                        if flag:
                            html_code += f'<a href="{self.get_term_url(full_term)}"><li class="avaliable">{hour.strftime("%H:%M")}</li></a>'
                        else:
                            html_code += f'<li class="avaliable">{hour.strftime("%H:%M")}</li>'
                    #html_code += f'<a href="{self.get_term_url(full_term)}"><li>{hour.strftime("%H:%M")}</li></a>'
                html_code += '</ul></li>'

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
        return reverse('client_app_new_visit', args=[self.username, self.service.id, year, week])

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
        return reverse('client_app_confirm_visit', args=[self.username, self.service.id,
                                                         term.year, term.month, term.day, term.hour, term.minute])

    def get_available_hours_in_day(self, date):
        visits = Visit.objects.filter(Q(user=self.user, client=self.client, start__year=date.year, start__month=date.month, start__day=date.day) |
                                      Q(user=self.user, client=self.client, end__year=date.year, end__month=date.month, end__day=date.day))

        start_work = timezone.make_aware(datetime.combine(date,self.work_time.start_time), timezone.get_default_timezone())
        end_work = timezone.make_aware(datetime.combine(date,self.work_time.end_time), timezone.get_default_timezone())

        avaliable_intervals = interval.closedopen(start_work, end_work)

        for visit in visits:
            visit = interval.closedopen(visit.start, visit.end)
            avaliable_intervals -= visit

        return_dict = {}
        list_of_hours = generate_hours_list(start_work, end_work, 15)

        for hour in list_of_hours:
            key = hour.strftime("%H:%M")
            if hour in avaliable_intervals:
                return_dict[key] = True
            else:
                return_dict[key] = False

        return return_dict

def generate_hours_list(start, end, step=15):
    list = []
    flag_time = start
    while flag_time <= end:
        list.append(flag_time)
        flag_time += timedelta(minutes=step)

    return list