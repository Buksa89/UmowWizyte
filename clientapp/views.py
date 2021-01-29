from datetime import datetime
from django.contrib.auth.models import User
from django.shortcuts import get_object_or_404, redirect, render
from django.utils import timezone
from django.utils.decorators import method_decorator
from django.views import View
from django.views.generic.edit import CreateView
from functools import wraps
from .forms import AddVisitForm, ClientChooseVisitForm, ClientLoginForm
from userapp.base import ClientAddVisitSchedule, is_client_authenticated, not_naive, get_user_from_url
from userapp.models import Client, Service, UserSettings, Visit, WorkTime

DAYS_OF_WEEK = ['Poniedziałek', 'Wtorek', 'Środa', 'Czwartek', 'Piątek', 'Sobota', 'Niedziela']

def client_login_required(function):
    @wraps(function)
    def wrap(request, *args, url_key, **kwargs):
        user = get_user_from_url(url_key)
        if is_client_authenticated(request, user.username):
            return function(request, *args, url_key, **kwargs)
        else:
            return redirect('client_login', url_key)
    return wrap



class ClientAppCancelVisit(View):
    def get(self, request, url_key, visit_id):
        user = get_user_from_url(url_key)
        client = get_object_or_404(Client, phone_number=request.session['client_authorized']['phone'], user=user)
        visit = get_object_or_404(Visit.objects.filter(user=user, client=client, id=visit_id, end__gt=not_naive(datetime.now())).exclude(is_available=False, is_confirmed=True))
        if visit.is_confirmed:
            visit.is_available = False
            visit.is_confirmed = False
            visit.save()
        elif visit.is_available:
            visit.delete()

        return redirect('client_dashboard', url_key)

class ClientAppLogout(View):
    def get(self, request, url_key):
        request.session.pop('client_authorized', None)
        return redirect('client_login', url_key)

class Dashboard(View):
    """In dashboard client see:
    Main menu (visits, settings, logout)
    Form to add new visit
    His future visits. He can cancel them
    Cancelled and confirmed visit are not display
    Old visit too"""

    template = 'client_dashboard.html'
    section = 'dashboard'

    @method_decorator(client_login_required)
    def get(self, request, url_key):
        user = get_user_from_url(url_key)
        client = get_object_or_404(Client, phone_number=request.session.get('client_authorized')['phone'],user=user)
        visits = Visit.objects.filter(user=user, client=client, end__gt=datetime.now()).exclude(is_available=False, is_confirmed=True)
        form = ClientChooseVisitForm(user)
        return render(request, self.template, {'form':form,
                                               'url_key':url_key,
                                               'username':user.username,
                                               'client_name':client.name,
                                               'visits':visits,
                                               'section':self.section})

    @method_decorator(client_login_required)
    def post(self, request, url_key):

        if 'service' in request.POST.keys(): return redirect('client_new_visit', url_key, request.POST['service'])

        return redirect('client_dashboard', url_key)

    def prepare_visits_list(self, visits):
        visits_list = []
        for visit in visits:
            dict = {}
            dict['name'] = visit.name
            dict['date'] = visit.start.strftime("%Y-%m-%d")
            dict['day'] = DAYS_OF_WEEK[visit.start.weekday()]
            dict['time'] = visit.start.strftime("%H:%M")
            dict['duration'] = str(visit.end - visit.start)[:-3].rjust(5,'0')
            if visit.is_available: dict['status'] = 'Aktualna'
            else: dict['status'] = 'Odwołana'
            if not visit.is_confirmed: dict['status'] += ' (Niepotwierdzona)'
            """dict['is_avaliable'] = visit.is_available
            dict['cancel_url'] = visit.get_cancel_url()"""
            visits_list.append(dict)
        return visits_list

class ClientLogin(CreateView):
    """If user is not active, login form is locked
    If client is authorized, he is redirected to dashboard
    If client give correct data, he start to be authorized in session:
    request.session['client_authorized'] = {'phone': phone_of_client, 'user': username
    This authorization is necessary to display dashboard
    When client is authorized, he is redirected to dashboard
    """
    template = 'client_login.html'
    template_not_active = 'client_login_not_active.html'

    def get(self, request, url_key):
        site = get_object_or_404(UserSettings, site_url=url_key)
        user = site.user
        if is_client_authenticated(request, user.username):
            return redirect('client_dashboard', url_key)

        if user.is_active:
            form = ClientLoginForm()
            return render(request, self.template, {'form':form, 'site':site})
        else:
            return render(request, self.template_not_active, {'site':site})


    def post(self, request, url_key):

        site = get_object_or_404(UserSettings, site_url=url_key)
        user = site.user

        if is_client_authenticated(request, user.username):
            return redirect('client_dashboard', url_key)

        form = ClientLoginForm(data=request.POST)
        if form.is_valid():
            cd = form.cleaned_data
            client = Client.objects.filter(phone_number=cd['phone_number'], user=user)
            form.clean()

            if not client:
                form.add_error(None, f'Numer nie jest zarejestrowany. Skontaktuj sie z {site.site_name}')
            elif client[0].pin != cd['pin']:
                form.add_error(None, 'Dane nieprawidłowe')
            elif not client[0].is_active:
                form.clean()
                form.add_error(None, 'Konto zablokowane')
            else:

                request.session['client_authorized'] = {'phone': cd['phone_number'], 'user': user.username}
                return redirect('client_dashboard', url_key)

        return render(request, self.template, {'form':form, 'site':site})

class NewVisit(View):
    template = 'client_new_visit.html'
    section = ''
    def get(self, request, url_key, service_id, year=datetime.now().year, week = datetime.now().isocalendar()[1]):
        user = get_user_from_url(url_key)
        client = get_object_or_404(Client, phone_number=request.session.get('client_authorized')['phone'],user=user)
        service = get_object_or_404(Service, id=service_id, user=user, is_active=True)

        schedule = ClientAddVisitSchedule(user, service, year, week)
        return render(request, self.template,
                      {'section': 'schedule', 'service': service.name,
                       'schedule': schedule.display(), 'url_key': url_key})

class ClientAppConfirmVisit(View):
    template = 'client_confirm_visit.html'
    section = 'dashboard'
    form = AddVisitForm()

    def get(self, request, url_key, service_id, year, month, day, hour, minute):

        user, service, date_, time_ = self.prepare_data(request, url_key, service_id, year, month, day, hour, minute)
        day_number = datetime(year, month, day).weekday()
        day_name = DAYS_OF_WEEK[day_number]

        return render(request, self.template, {'section': self.section, 'service': service, 'date': date_, 'time': time_,
                                                                 'day_name': day_name, 'form':self.form,
                                                                 'url_key': url_key, 'service_id': service_id,
                                                                 'year':year, 'month':month, 'day':day, 'hour':hour,
                                                                 'minute':minute})

    def post(self, request, url_key, service_id, year, month, day, hour, minute):
        user, service, date_, time_ = self.prepare_data(request, url_key, service_id, year, month, day, hour, minute)
        day_number = datetime(year, month, day).weekday()
        day_name = DAYS_OF_WEEK[day_number]

        form = AddVisitForm(data=self.request.POST)

        client = get_object_or_404(Client, phone_number=request.session.get('client_authorized')['phone'], user=user)
        start = datetime(year, month, day, hour, minute)
        end = start + service.duration

        try:
            form.save(user, client, service.name, start, end, True, False)
            return redirect('client_dashboard', url_key=url_key)
        except:
            pass

        return render(request, self.template, {'section': self.section, 'service': service, 'date': date_, 'time': time_,
                                                                 'day_name': day_name, 'form':self.form,
                                                                 'url_key': url_key, 'service_id': service_id,
                                                                 'year':year, 'month':month, 'day':day, 'hour':hour,
                                                                 'minute':minute})

    def prepare_data(self, request, url_key, service_id, year, month, day, hour, minute):
        user = get_user_from_url(url_key)
        service = get_object_or_404(Service, id=service_id, user=user)
        dt = datetime(year, month, day, hour, minute)
        date_ = dt.strftime("%y-%m-%d")
        time_ = dt.strftime("%H:%M")

        return user, service, date_, time_