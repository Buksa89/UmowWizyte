from datetime import datetime
from django.contrib.auth.models import User
from django.shortcuts import get_object_or_404, redirect, render
from django.utils import timezone
from django.utils.decorators import method_decorator
from django.views import View
from django.views.generic.edit import CreateView
from functools import wraps
from .forms import AddVisit, ClientChooseVisitForm, ClientLoginForm
from userapp.base import ClientAddVisitSchedule, is_client_authenticated
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
            if visit.is_available: dict['status'] = 'Zarezerwowana'
            else: dict['status'] = 'Odwołana'
            if visit.is_confirmed: dict['status'] += ' (Potwierdzona)'
            else: dict['status'] += ' (Niepotwierdzona)'
            dict['is_avaliable'] = visit.is_available
            dict['cancel_url'] = visit.get_cancel_url()
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




class ClientAppNewVisit(View):
    template_name = ''
    section = ''
    def get(self, request, username, service_id, year=datetime.now().year, week = datetime.now().isocalendar()[1]):
        user = get_object_or_404(User, username__iexact=username)
        client = get_object_or_404(Client, phone_number=request.session.get('client_authorized')['phone'],user=user)
        service = get_object_or_404(Service, id=service_id, user=user, is_active=True)
        work_time = get_object_or_404(WorkTime, user=user)
        available_dates = ''
        schedule = ClientAddVisitSchedule(username)
        return render(request, 'client_new_visit.html',
                      {'section': 'schedule', 'service': service.name,
                       'schedule': schedule.display(year, week, service), 'username': username})


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
        start = timezone.make_aware(datetime(year, month, day, hour, minute), timezone.get_default_timezone())
        end = start + service.duration
        is_available = True
        is_confirmed = False

        form.save(user, client, name, start, end, is_available, is_confirmed)

        return redirect('client_app_dashboard', username)


class ClientAppCancelVisit(View):

    def get(self, request, username, visit_id):
        user = get_object_or_404(User, username__iexact=username)
        client = get_object_or_404(Client, phone_number=request.session['client_authorized']['phone'], user=user)
        visit = get_object_or_404(Visit, user=user, client=client, id=visit_id)
        if visit.is_confirmed:
            visit.is_available = False
            visit.is_confirmed = False
            visit.save()
        elif visit.is_available:
            visit.delete()

        return redirect('client_app_dashboard', username)

class ClientAppLogout(View):
    def get(self, request, username):
        request.session.pop('client_authorized', None)
        return redirect('client_app_login', username)



