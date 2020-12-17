from datetime import date, datetime, time, timedelta
from django.contrib.auth import authenticate, login
from django.contrib.auth.decorators import login_required
from django.utils.decorators import method_decorator
from django.contrib.auth.models import User
from django.db import transaction
from django.db.utils import IntegrityError
from django.shortcuts import get_object_or_404, redirect, render
from django.urls import reverse
from django.utils.decorators import method_decorator
from django.views import View
from django.views.generic.edit import CreateView
from .base import DAYS_FOR_CODE, not_naive, UserAddVisitSchedule, UserLockTimeSchedule, UserSchedule, UserTwoDaysSchedule
from .forms import AddClientForm, AddServiceForm, AddVisitForm, LoginForm, NewVisitForm, WorkTimeForm
from .models import Client, Service, Visit, WorkTime




""" User Views """

class Dashboard(CreateView):

    """ In Dashboard User can see all not-confirmed visit, confirm or reject them
    Also he can see today schedule and navigate to previous or next day schedule".
    In this view is form to add new visit by user."""

    template_name = 'dashboard.html'
    section = 'dashboard'

    @method_decorator(login_required)
    def get(self, request, year=datetime.now().year, month=datetime.now().month, day=datetime.now().day):
        visits_list = []
        user = User.objects.get(username=request.user)
        visits = Visit.objects.filter(user=user, is_confirmed=False)
        form = NewVisitForm(user=user)

        for visit in visits:
            client = [visit.client.name, visit.client.surname, visit.client.phone_number]
            client = '<br />'.join(client)
            datetime_ = visit.start.strftime('%y-%m-%d<br />%H:%M')
            reject_url = visit.get_reject_url()
            confirm_url = visit.get_confirm_url()
            if visit.is_available: status = "Nowa"
            else: status = "<b>Odwołana</b>"

            visits_list.append({'id':visit.id, 'client':client, 'name':visit.name, 'status':status, 'date':datetime_,
                                'description':visit.description, 'reject_url':reject_url, 'confirm_url':confirm_url})

        schedule = UserTwoDaysSchedule(request.user, year, month, day)

        return render(request, self.template_name, {'section':self.section, 'visits':visits_list,
                                                    'schedule': schedule.display(), 'form':form})

    @method_decorator(login_required)
    def post(self, request):
        service = get_object_or_404(Service, user=request.user, id=request.POST['service'])
        if request.POST['duration'] == '00:00': duration = service.display_duration()
        else: duration = request.POST['duration']

        hours = int(duration[:2])
        minutes = int(duration[-2:])

        return redirect(reverse('dashboard_new_visit', args=[request.POST['client'], request.POST['service'], hours, minutes]))


class DashboardLockTime(CreateView):

    template_name = 'schedule.html'
    section = 'dashboard'

    @method_decorator(login_required)
    def get(self, request, year=datetime.now().year, week=None):
        if not week:
            week = datetime.now().isocalendar()[1]
        schedule = UserLockTimeSchedule(request.user)

        return render(request, self.template_name, {'section': self.section, 'schedule': schedule.display(year, week)})

class DashboardVisit(CreateView):

    template_name = 'visit.html'
    section = 'dashboard'

    @method_decorator(login_required)
    def get(self, request, visit_id):
        user = User.objects.get(username=request.user)
        visit = get_object_or_404(Visit, user=user, id=visit_id)

        return render(request, self.template_name, {'section': self.section, 'visit': visit})


class DashboardVisitCancel(CreateView):

    template_name = 'visit.html'
    section = 'dashboard'

    @method_decorator(login_required)
    def get(self, request, visit_id):
        user = User.objects.get(username=request.user)
        visit = get_object_or_404(Visit, user=user, id=visit_id)
        visit.is_available = False
        visit.is_confirmed = True
        visit.save()
        return redirect('dashboard')


class DashboardNewVisit(CreateView):

    template_name = 'schedule.html'
    section = 'dashboard'

    @method_decorator(login_required)
    def get(self, request, client_id, service_id, hours, minutes, year=datetime.now().year, week=False):
        if not week:
            week = datetime.now().isocalendar()[1]
        user = User.objects.get(username=request.user)
        client = get_object_or_404(Client, user=user, id=client_id)
        service = get_object_or_404(Service, user=user, id=service_id)
        duration = timedelta(hours=hours, minutes=minutes)
        schedule = UserAddVisitSchedule(request.user, year, week, client, service, duration)

        return render(request, self.template_name, {'section': self.section,
                                                    'schedule': schedule.display()})

class DashboardConfirmVisit(CreateView):

    template_name = 'confirm_visit.html'
    section = 'dashboard'
    #TODO Walidacje, wspolna czesc kodu dla get i post
    @method_decorator(login_required)
    def get(self, request, client_id, service_id, hours, minutes, year, month, day, hour, minute):

        user = User.objects.get(username=request.user)
        client = get_object_or_404(Client, user=user, id=client_id)
        service = get_object_or_404(Service, user=user, id=service_id)
        start = not_naive(datetime(year, month, day, hour, minute))
        form = AddVisitForm()

        data = {'client': client, 'service': service, 'start_date': start.strftime("%y-%m-%d"), 'start_time': start.strftime("%H:%M"), 'duration': f'{hours}:{minutes}'}
        url_data = {'client_id': client_id, 'service_id':service_id , 'hours':hours, 'minutes':minutes, 'year':year,
                'month':month, 'day':day, 'hour':hour, 'minute':minute}

        return render(request, self.template_name, {'form': form,
                                                    'section': self.section,
                                                    'data': data, 'url_data':url_data})

    @method_decorator(login_required)
    def post(self, request, client_id, service_id, hours, minutes, year, month, day, hour, minute):

        user = User.objects.get(username=request.user)
        client = get_object_or_404(Client, user=user, id=client_id)
        service = get_object_or_404(Service, user=user, id=service_id)
        start = not_naive(datetime(year, month, day, hour, minute))
        end = start + timedelta(hours=hours, minutes=minutes)
        form = AddVisitForm(data=self.request.POST)
        data = {'client': client, 'service': service, 'start_date': start.strftime("%y-%m-%d"), 'start_time': start.strftime("%H:%M"), 'duration': f'{hours}:{minutes}', 'description':self.request.POST['description']}
        url_data = {'client_id': client_id, 'service_id':service_id , 'hours':hours, 'minutes':minutes, 'year':year,
                'month':month, 'day':day, 'hour':hour, 'minute':minute}

        try:
            form.save(user, client, service.name, start, end)
            return render(request, self.template_name, {'section': self.section,
                                                    'data': data, 'url_data': url_data, 'saved': True})
        except:
            return render(request, self.template_name, {'form': form,
                                                        'section': self.section,
                                                        'data': data, 'url_data':url_data})

class DashboardVisitReject(CreateView):

    @method_decorator(login_required)
    def get(self, request, visit_id):
        user = User.objects.get(username=request.user)
        visit = get_object_or_404(Visit, id=visit_id,user=user)
        if visit.is_available:
            visit.delete()
        else:
            visit.is_available = True
            visit.save()

        return redirect('dashboard')


class DashboardVisitConfirm(CreateView):

    @method_decorator(login_required)
    def get(self, request, visit_id):
        user = User.objects.get(username=request.user)
        visit = get_object_or_404(Visit, id=visit_id,user=user)
        visit.is_confirmed = True
        visit.save()
        return redirect('dashboard')


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
    section = 'dashboard_schedule'

    @method_decorator(login_required)
    def get(self, request, year=datetime.now().year, week=False, month=False, day=False):
        user = User.objects.get(username=request.user)
        form = NewVisitForm(user=user)
        if not week: week = datetime.now().isocalendar()[1]

        if not day:
            schedule = UserSchedule(request.user, year, week)
            return render(request, self.template_schedule_name,
                          {'section': self.section, 'schedule': schedule.display(), 'form':form})
        else:
            schedule = UserOneDaySchedule(request.user)
            return render(request, self.template_schedule_name,
                          {'section': self.section, 'schedule': schedule.display(year, month, day), 'form':form})



class DashboardSettings(CreateView):
    template_name = 'settings.html'



    @method_decorator(login_required)
    def get(self, request):
        user, service_form, work_time_form = self.settings_prepare_data(request)
        services = self.settings_prepare_services(user)
        return render(request, 'settings.html', {'work_time_form': work_time_form,
                                                        'service_form': service_form,
                                                       'services': services,
                                                       'section':'dashboard_settings'})

    @method_decorator(login_required)
    def post(self, request):
        created_service = None
        work_time_changed = False
        user, service_form, work_time_form = self.settings_prepare_data(request)

        if 'submit' not in request.POST.keys(): None
        elif request.POST['submit'] == 'add_service':
            service_form, created_service = self.dashboard_settings_services(request, user)
        elif request.POST['submit'] == 'set_work_time':
            work_time_form, work_time_changed = self.dashboard_settings_work_time(request, user)

        services = self.settings_prepare_services(user)




        return render(request, 'settings.html', {'work_time_form': work_time_form,
                                                        'service_form': service_form,
                                                       'services': services,
                                                       'created_service': created_service,
                                                       'work_time_changed': work_time_changed,
                                                       'section':'dashboard_settings'})

    def settings_prepare_services(self, user):
        services = Service.objects.filter(user=user)
        services = self.generate_service_list(services)
        return services

    def settings_prepare_data(self, request):
        user = User.objects.get(username=request.user)
        service_form = AddServiceForm()
        work_time = WorkTime.objects.get(user=user)



        for day in DAYS_FOR_CODE:
            end = datetime.combine(date.min, work_time.__dict__['start_'+day]) + work_time.__dict__['duration_'+day]
            if end >= datetime(1,1,2,0,0,0): work_time.__dict__['end_'+day] = '24:00'
            else: work_time.__dict__['end_'+day] = end.time().strftime("%H:%M")
            work_time.__dict__['start_'+day] = work_time.__dict__['start_'+day].strftime("%H:%M")


        work_time_form = WorkTimeForm(initial=work_time.__dict__, instance=work_time)
        return user, service_form, work_time_form

    def generate_service_list(self, services):
        service_list = []
        for service in services:
            dict = {}
            dict['name'] = service.name
            dict['display_duration'] = service.display_duration
            dict['status'] = service.is_active
            dict['get_remove_url'] = service.get_remove_url
            dict['get_lock_url'] = service.get_lock_url
            service_list.append(dict)
        return service_list

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
        duration_for_days = []
        for day in DAYS_FOR_CODE:
            hours, minutes = request.POST['end_'+day].split(':')
            hours = int(hours)
            minutes = int(minutes)
            if hours < 24:
                days = 1
                hours = hours
            else:
                days = hours // 24 + 1
                hours = hours % 24
            end_time = datetime(1,1, days, hours, minutes)

            hours, minutes = request.POST['start_'+day].split(':')
            start_time = datetime(1,1, 1, int(hours), int(minutes))
            duration_for_days.append(end_time - start_time)

        form = WorkTimeForm(data=request.POST, instance=work_time)
        if form.is_valid():
            form.save(duration_for_days)
            return form, True
        return form, False



@login_required
def dashboard_settings_service_remove(request,service_id):
    #TODO: Dodaj potwierdzenie usunięcia
    user = User.objects.get(username=request.user)
    get_object_or_404(Service, id=service_id,user=user).delete()
    return redirect('dashboard_settings')

class DashboardSettingsServiceLock(CreateView):

    @method_decorator(login_required)
    def get(self, request, service_id):
        user = User.objects.get(username=request.user)
        service = get_object_or_404(Service, id=service_id,user=user)
        if service.is_active:
            service.is_active = False
            service.save()
        else:
            service.is_active = True
            service.save()

        return redirect('dashboard_settings')



def login_screen(request):
    if request.user.is_authenticated:
        return redirect('dashboard')
    else:
        if request.method == 'POST':
            form = LoginForm(request.POST)
            if form.is_valid():
                cd = form.cleaned_data
                user = authenticate(username=cd['username'],
                                    password=cd['password'])
                if user:
                    login(request, user)
                    return redirect('dashboard')
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
    users = User.objects.filter(is_superuser=False)
    return render(request, 'welcome.html', {'users':users})