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
from .variables import DAYS_OF_WEEK, DAYS_FOR_CODE
from .base import not_naive, UserAddVisitSchedule, UserLockTimeSchedule, Schedule, UserTwoDaysSchedule, td_to_string
from .forms import AddClientForm, AddServiceForm, AddVisitForm, ContactForm, EditClientForm, WorkHolidaysForm, LoginForm, NewVisitForm, \
    RegistrationForm, UserEditForm, UserPassForm, UserSettingsForm, NewWorkTimeForm
from .models import Client, Service, UserSettings, Visit, WorkTime
import userapp.language as l




""" Nieaktualne """

class Dashboard2(CreateView):
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

class DashboardConfirmVisit(CreateView):

    template_name = 'dashboard_new_visit_3.html'
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






""" Dashboard New Visit Step 3 """
class DashboardNewVisit3(View):
    template = 'dashboard_new_visit_3.html'
    section = 'dashboard'
    subsection = 'new_visit'

    @method_decorator(login_required)
    def get(self, request, client_id, service_id, hours, minutes, year, month, day, hour, minute):
        user = User.objects.get(username=request.user)
        client = get_object_or_404(Client, user=user, id=client_id)
        service = get_object_or_404(Service, user=user, id=service_id)
        start = not_naive(datetime(year, month, day, hour, minute))
        form = AddVisitForm()
        #TODO: Walidacja, czy czas wolny, połączenie fragmentu kodu z get i post
        data = {'client': client, 'service': service, 'start_date': start.strftime("%y-%m-%d"), 'start_time': start.strftime("%H:%M"), 'duration': f'{hours}:{minutes}'}
        url_data = {'client_id': client_id, 'service_id':service_id , 'hours':hours, 'minutes':minutes, 'year':year,
                'month':month, 'day':day, 'hour':hour, 'minute':minute}

        return render(request, self.template, {'form': form,
                                               'subsection':self.subsection,
                                               'section': self.section,
                                               'data': data,
                                               'url_data':url_data})

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
            return redirect('dashboard')
        except:
            pass

        return render(request, self.template, {'form': form,
                                               'subsection':self.subsection,
                                               'section': self.section,
                                               'data': data,
                                               'url_data':url_data})

""" Dashboard New Visit Step 2 """
class DashboardNewVisit2(View):
    template = 'dashboard_new_visit_2.html'
    section = 'dashboard'
    subsection = 'new_visit'

    @method_decorator(login_required)
    def get(self, request, client_id, service_id, hours, minutes, year=datetime.now().year, week=False):
        if not week:
            week = datetime.now().isocalendar()[1]
        user = User.objects.get(username=request.user)
        client = get_object_or_404(Client, user=user, id=client_id)
        service = get_object_or_404(Service, user=user, id=service_id)
        duration = timedelta(hours=hours, minutes=minutes)
        schedule = UserAddVisitSchedule(request.user, year, week, client, service, duration)

        return render(request, self.template, {'section': self.section,
                                               'subsection':self.subsection,
                                               'schedule': schedule.display()})

""" Dashboard New Visit Step 1 """
class DashboardNewVisit1(View):
    template = 'dashboard_new_visit_1.html'
    section = 'dashboard'
    subsection = 'new_visit'

    @method_decorator(login_required)
    def get(self, request):
        user = User.objects.get(username=request.user)
        form = NewVisitForm(user=user)

        return render(request, self.template, {'form': form,
                                               'section': self.section,
                                               'subsection':self.subsection})

    @method_decorator(login_required)
    def post(self, request):
        service = get_object_or_404(Service, user=request.user, id=request.POST['service'])
        if request.POST['duration'] == '0:00:00': duration = service.display_duration()
        else: duration = request.POST['duration']
        print(f'dur:{duration}')
        splitted_duration = duration.split(":")
        hours = splitted_duration[0]
        minutes = splitted_duration[1]

        return redirect(reverse('dashboard_new_visit_2', args=[request.POST['client'], request.POST['service'], hours, minutes]))

""" Dashboard """
class Dashboard(View):
    template = 'dashboard.html'
    section = 'dashboard'
    subsection = 'today'

    @method_decorator(login_required)
    def get(self, request, year=datetime.now().year, month=datetime.now().month, day=datetime.now().day):
        user = User.objects.get(username=request.user)
        #TODO: Przygotuj liste powiadomien!
        visits = Visit.objects.filter(user=user, is_confirmed=False)
        schedule = UserTwoDaysSchedule(request.user, year, month, day)

        return render(request, self.template, {'section': self.section,
                                               'subsection':self.subsection,
                                               'visits': visits,
                                               'schedule': schedule.display(),
                                               })

""" Schedule """
class MainSchedule(View):
    template = 'schedule.html'
    section = 'schedule'

    @method_decorator(login_required)
    def get(self, request, year=datetime.now().year, week=False):
        user = User.objects.get(username=request.user)
        if not week: week = datetime.now().isocalendar()[1]
        schedule = Schedule(request.user, year, week)

        return render(request, self.template, {'section': self.section,
                                               'schedule': schedule.display()})

""" Clients """
class Clients(CreateView):
    template = 'clients.html'
    section = 'clients'
    subsection = 'list'

    @method_decorator(login_required)
    def get(self, request):
        #TODO: Wszedzie user.get zamien na user get or 404
        user = User.objects.get(username=request.user)
        clients = Client.objects.filter(user=user)

        return render(request, self.template, {"clients": clients,
                                               'section':self.section,
                                               'subsection':self.subsection})

""" Clients Add """

class ClientsAdd(CreateView):
    template_name = 'clients_add.html'
    section = 'clients'
    subsection = 'add'

    @method_decorator(login_required)
    def get(self, request):
        form = AddClientForm()
        return render(request, self.template_name, {'form': form,
                                                    'section':self.section,
                                                    'subsection':self.subsection})

    @method_decorator(login_required)
    def post(self, request):
        form = AddClientForm(data=self.request.POST)
        if form.is_valid():
            user = User.objects.get(username=request.user)
            form.save(user)
            messages.add_message(request, messages.INFO, 'Klient dodany')
            #TODO Wiedomości sukces powinny być dodawane tak:
            # messages.success(self.request, f'<p>{request.POST.get("name","")} dodany.</p>')
            form = AddClientForm()

        return render(request, self.template_name, {'form': form,
                                                    'section':self.section,
                                                    'subsection':self.subsection})

""" Clients Remove """

class ClientsRemove(CreateView):

    @method_decorator(login_required)
    def get(self, request, client_id):
        user = User.objects.get(username=request.user)
        get_object_or_404(Client, id=client_id,user=user).delete()
        return redirect('clients')

""" Clients Data """

class ClientsData(CreateView):

    template = 'clients_data.html'
    section = 'clients'
    subsection = 'data'

    @method_decorator(login_required)
    def get(self, request, client_id):
        user = User.objects.get(username=request.user)
        client = get_object_or_404(Client, id=client_id,user=user)
        visits = Visit.objects.filter(client=client)
        return render(request, self.template, {'client': client,
                                                    'visits':visits,
                                                    'section':self.section,
                                                    'subsection':self.subsection})
    #TODO: WIZYTY!!!!

""" Clients Edit """

class ClientsEdit(CreateView):

    template = 'clients_edit.html'
    success_template = 'clients_data.html'
    section = 'clients'
    subsection = 'edit'

    @method_decorator(login_required)
    def get(self, request, client_id):
        user = User.objects.get(username=request.user)
        client = get_object_or_404(Client, id=client_id,user=user)
        form = EditClientForm(instance=client)
        return render(request, self.template, {'form': form,
                                                    'client_id': client.id,
                                                    'section':self.section,
                                                    'subsection':self.subsection})

    @method_decorator(login_required)
    def post(self, request, client_id):
        user = User.objects.get(username=request.user)
        client = get_object_or_404(Client, id=client_id,user=user)
        form = EditClientForm(data=self.request.POST, instance=client)
        if form.is_valid():
            form.save()
            return redirect('clients_data', client.id)

        return render(request, self.template, {'form': form,
                                                    'client_id': client.id,
                                                    'section':self.section,
                                                    'subsection':self.subsection})

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
        form_user = UserEditForm(instance=user, data=request.POST)
        form_pass = UserPassForm(data=request.POST)
        form_settings = UserSettingsForm(instance=user_settings, data=request.POST)

        #TODO: Znajdź lepsze rozwiązanie
        # if form_user.is_valid() and form_settings.is_valid() and form_pass.is_valid() nie działa, bo
        # zapisuje puste hasło nawet bez form.pass.save()
        # Tutaj rozwiazalem przez ponowne utworzenie formularza, tym razem z instancją


        if form_user.is_valid() and form_settings.is_valid() and form_pass.is_valid():

            form_user.save()
            form_settings.save()
            fp = form_pass.cleaned_data
            if fp['password'] or fp['password2']:
                form_pass = UserPassForm(instance=user, data=request.POST)
                new_pass = form_pass.save(commit=False)
                new_pass.set_password(form_pass.cleaned_data['password'])
                new_pass.save()

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
    work_time_form = NewWorkTimeForm()

    @method_decorator(login_required)
    def get(self, request):
        user = User.objects.get(username=request.user)
        work_times = self.work_time_display(user)
        user_settings = UserSettings.objects.get(user=user)
        work_holidays_form = WorkHolidaysForm(instance=user_settings)

        return render(request, self.template, {'work_times': work_times,
                                               'work_time_form': self.work_time_form,
                                               'work_holidays_form': work_holidays_form,
                                               'section':self.section,
                                               'subsection':self.subsection})

    @method_decorator(login_required)
    def post(self, request):
        user = User.objects.get(username=request.user)
        user_settings = UserSettings.objects.get(user=user)
        work_holidays_form = WorkHolidaysForm(instance=user_settings)
        if request.POST['submit'] == 'work_time':
            self.work_time_form = NewWorkTimeForm(data=request.POST)
            if self.work_time_form.is_valid():
                cd = self.work_time_form.cleaned_data
                self.work_time_form.save(user)
                messages.add_message(request, messages.INFO, f'Czas pracy dodany.')
                self.work_time_form = NewWorkTimeForm()
        if request.POST['submit'] == 'holidays':
            work_holidays_form = WorkHolidaysForm(data=request.POST, instance=user_settings)
            if work_holidays_form.is_valid():
                cd = work_holidays_form.cleaned_data
                messages.add_message(request, messages.INFO, f'Czas pracy zmeiniony.')
                work_holidays_form.save(user)
                work_holidays_form = WorkHolidaysForm(instance=user_settings)


        work_times = self.work_time_display(user)
        return render(request, self.template, {'work_times': work_times,
                                               'work_time_form': self.work_time_form,
                                               'work_holidays_form': work_holidays_form,
                                               'section':self.section,
                                               'subsection':self.subsection})

    def work_time_display(self, user):
        work_times = WorkTime.objects.filter(user=user).order_by('day_of_week','start','end')

        for work_time in work_times:
            work_time.start = td_to_string(work_time.start)
            work_time.end = td_to_string(work_time.end)
            work_time.day_of_week = DAYS_OF_WEEK[work_time.day_of_week]

        return work_times

class SettingsWorkTimeRemove(CreateView):
    @method_decorator(login_required)
    def get(self, request, work_time_id):
        user = User.objects.get(username=request.user)
        work_time = get_object_or_404(WorkTime, id=work_time_id, user=user)
        work_time.delete()
        return redirect('settings_work_time')



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

class Registration(CreateView):
    template = 'registration.html'
    form = RegistrationForm()

    def get(self, request):
        return render(request, self.template, {'form': self.form, 'l':l})

    def post(self, request):
        self.form = RegistrationForm(request.POST)
        if self.form.is_valid():
            new_user = self.form.save(commit=False)
            new_user.set_password(self.form.cleaned_data['password'])
            new_user.save()
            login_template = 'login.html'
            form = LoginForm()
            messages.success(request, l.WELCOME_ABOARD)
            return render(request, login_template, {'form':form, 'l':l})


        return render(request, self.template, {'form': self.form, 'l':l})

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
                return redirect('dashboard')
            else:
                self.form.clean()
                self.form.add_error(None, 'Błędny login lub hasło')

        return render(request, self.template, {'form': self.form})

class PasswordResetComplete(CreateView):
    template = 'login.html'
    form = LoginForm()

    def get(self, request):
        messages.success(request, 'Profile details updated.')
        messages.add_message(request, messages.INFO, 'Hasło zmienione!<br />Możesz się teraz zalogować')
        return render(request, self.template, {'form':self.form})


