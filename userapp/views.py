from datetime import date, datetime, time, timedelta
from django.core.mail import send_mail
from django.contrib import messages
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
from .forms import AddClientForm, AddServiceForm, AddVisitForm, ContactForm, EditClientForm, LoginForm, NewVisitForm, \
    RegistrationForm, UserEditForm, UserPassForm, UserSettingsForm, WorkTimeForm
from .models import Client, Service, UserSettings, Visit, WorkTime




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
            messages.success(self.request, f'<p>{request.POST.get("name","")} dodany.</p>'
                                           f'<p><a href="{reverse("dashboard_clients")}">Kliknij tutaj</a> aby powrócić do listy klientów</p>')
            form = AddClientForm()
            return render(request, self.template_name, {'form': form,
                                                        'section':self.section})

        return render(self.request, self.template_name, {'form': form, 'section': 'dashboard_clients'})

class DashboardClientsEdit(CreateView):
    template_name = 'clients_edit.html'
    section = 'dashboard_clients'

    @method_decorator(login_required)
    def get(self, request, client_id):
        user = User.objects.get(username=request.user)
        client = get_object_or_404(Client, id=client_id,user=user)
        form = EditClientForm(instance=client)
        # dane w formularzu
        # wizyty
        # historia
        return render(request, self.template_name, {'form': form, 'client_id': client.id, 'section':self.section})

    @method_decorator(login_required)
    def post(self, request, client_id):
        user = User.objects.get(username=request.user)
        client = get_object_or_404(Client, id=client_id,user=user)
        form = EditClientForm(data=self.request.POST, instance=client)
        if form.is_valid():
            form.save()
            messages.success(self.request, f'Dane klienta zostały zmienione')
            form = EditClientForm(instance=client)
            return render(request, self.template_name, {'form': form,
                                                        'client_id': client.id,
                                                        'section': self.section})

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






""" Settings Account """
class SettingsAccount(CreateView):
    template = 'settings_account.html'
    section = 'settings'
    subsection = 'account'

    @method_decorator(login_required)
    def get(self, request):
        user = User.objects.get(username=request.user)
        user_settings = UserSettings.objects.get(user=user)
        form_user = UserEditForm(instance=user)
        form_pass = UserPassForm()
        form_settings = UserSettingsForm(instance=user_settings)

        return render(request, self.template, {'form_user': form_user,
                                               'form_pass': form_pass,
                                               'form_settings': form_settings,
                                               'section':self.section,
                                               'subsection':self.subsection})

    @method_decorator(login_required)
    def post(self, request):
        user = User.objects.get(username=request.user)
        user_settings = UserSettings.objects.get(user=user)
        form_user = UserEditForm(data=request.POST, instance=user)
        form_pass = UserPassForm(data=request.POST, instance=user)
        form_settings = UserSettingsForm(data=request.POST, instance=user_settings)


        if form_user.is_valid() and form_settings.is_valid() and form_pass.is_valid():

            form_user.save()
            form_settings.save()
            fp = form_pass.cleaned_data
            if fp['password'] or fp['password2']:
                new_user = form_pass.save(commit=False)
                new_user.set_password(form_pass.cleaned_data['password'])
                new_user.save()

            messages.add_message(request, messages.INFO, 'Dane zostały zmienione')




        return render(request, self.template, {'form_user': form_user,
                                               'form_pass': form_pass,
                                               'form_settings': form_settings,
                                               'section':self.section,
                                               'subsection':self.subsection})


""" Settings Contact """
class SettingsContact(CreateView):
    template = 'settings_contact.html'
    section = 'settings'
    subsection = 'contact'
    form = ContactForm()

    @method_decorator(login_required)
    def get(self, request):

        return render(request, self.template, {'form': self.form,
                                               'section':self.section,
                                               'subsection':self.subsection})

    @method_decorator(login_required)
    def post(self, request):
        user = User.objects.get(username=request.user)
        admin = User.objects.get(username='admin')
        self.form = ContactForm(data=request.POST)
        if self.form.is_valid():
            cd = self.form.cleaned_data
            send_mail(
                'Kontakt - Umawianie wizyt',
                cd['content'],
                user.email,
                [admin.email],
                fail_silently=False,
            )
            self.form = ContactForm()
            messages.add_message(request, messages.INFO, 'Wiadomość została wysłana')


        return render(request, self.template, {'form': self.form,
                                               'section':self.section,
                                               'subsection':self.subsection})

""" Settings WorkTime """

class SettingsWorkTime(CreateView):
    template = 'settings_work_time.html'
    section = 'settings'
    subsection = 'work_time'

    def prepare_data(self, request):
        user = User.objects.get(username=request.user)
        work_time = WorkTime.objects.get(user=user)

        for day in DAYS_FOR_CODE:
            end = datetime.combine(date.min, work_time.__dict__['start_'+day]) + work_time.__dict__['duration_'+day]
            if end >= datetime(1,1,2,0,0,0): work_time.__dict__['end_'+day] = '24:00'
            else: work_time.__dict__['end_'+day] = end.time().strftime("%H:%M")
            work_time.__dict__['start_'+day] = work_time.__dict__['start_'+day].strftime("%H:%M")


        work_time_form = WorkTimeForm(initial=work_time.__dict__, instance=work_time)
        return user, work_time_form

    @method_decorator(login_required)
    def get(self, request):
        user, self.form = self.prepare_data(request)
        return render(request, self.template, {'form': self.form,
                                               'section':self.section,
                                               'subsection':self.subsection})

    @method_decorator(login_required)
    def post(self, request):
        user, self.form = self.prepare_data(request)
        work_time = WorkTime.objects.get(user=user)
        duration_for_days = []

        for day in DAYS_FOR_CODE:
            hours, minutes = request.POST['end_' + day].split(':')
            hours = int(hours)
            minutes = int(minutes)
            if hours < 24:
                days = 1
                hours = hours
            else:
                days = hours // 24 + 1
                hours = hours % 24
            end_time = datetime(1, 1, days, hours, minutes)

            hours, minutes = request.POST['start_' + day].split(':')
            start_time = datetime(1, 1, 1, int(hours), int(minutes))
            duration_for_days.append(end_time - start_time)

        self.form = WorkTimeForm(data=request.POST, instance=work_time)

        if self.form.is_valid():
            messages.add_message(request, messages.INFO, f'Czas pracy zmieniony.')
            self.form.save(duration_for_days)

        return render(request, self.template, {'form': self.form,
                                               'section':self.section,
                                               'subsection':self.subsection})

""" Settings Services """

class SettingsServices(CreateView):
    template = 'settings_services.html'
    section = 'settings'
    subsection = 'services'
    form = AddServiceForm()

    @method_decorator(login_required)
    def get(self, request):
        user = User.objects.get(username=request.user)
        services = Service.objects.filter(user=user)
        return render(request, self.template, {'form': self.form,
                                               'services': services,
                                               'section':self.section,
                                               'subsection':self.subsection})

    @method_decorator(login_required)
    def post(self, request):
        user = User.objects.get(username=request.user)
        self.form = AddServiceForm(data=request.POST)
        if self.form.is_valid():
            cd = self.form.cleaned_data
            if Service.objects.filter(name=cd['name'], user=user):
                self.form.add_error(None, 'Masz już taką usługę ;)')
            else:
                messages.add_message(request, messages.INFO, f'Usługa dodana.')
                self.form.save(user)
                self.form = AddServiceForm()

        services = Service.objects.filter(user=user)

        return render(request, self.template, {'form': self.form,
                                               'services': services,
                                               'section':self.section,
                                               'subsection':self.subsection})

class SettingsServiceLock(CreateView):

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
        return redirect('settings')

class SettingsServiceRemove(CreateView):
    @method_decorator(login_required)
    def get(self, request, service_id):
        user = User.objects.get(username=request.user)
        service = get_object_or_404(Service, id=service_id,user=user)
        service.delete()
        return redirect('settings_services')

""" Other Views """

class Welcome(CreateView):
    template = 'welcome.html'
    def get(self, request):
        users = User.objects.filter(is_superuser=False)
        return render(request, self.template, {'users':users})

class Register(CreateView):
    template = 'register.html'
    form = RegistrationForm()

    def get(self, request):
        return render(request, self.template, {'form': self.form})

    def post(self, request):
        self.form = RegistrationForm(request.POST)
        if self.form.is_valid():
            new_user = self.form.save(commit=False)
            new_user.set_password(self.form.cleaned_data['password'])
            new_user.save()
            login_template = 'login.html'
            form = LoginForm()
            messages.add_message(request, messages.INFO, 'Witaj na pokładzie!<br />Możesz się teraz zalogować')
            return render(request, login_template, {'form':form})


        return render(request, self.template, {'form': self.form})

class Login(CreateView):
    template = 'login.html'
    form = LoginForm()

    def get(self, request):
        return render(request, self.template, {'form': self.form})

    def post(self, request):
        self.form = LoginForm(request.POST)
        if self.form.is_valid():
            cd = self.form.cleaned_data
            user = authenticate(username=cd['username'],
                                password=cd['password'])
            if user:
                login(request, user)
                return redirect('settings')
            else:
                self.form.clean()
                self.form.add_error(None, 'Błędny login lub hasło')

        return render(request, self.template, {'form': self.form})

class PasswordResetComplete(CreateView):
    template = 'login.html'
    form = LoginForm()

    def get(self, request):
        messages.add_message(request, messages.INFO, 'Hasło zmienione!<br />Możesz się teraz zalogować')
        return render(request, self.template, {'form':self.form})


