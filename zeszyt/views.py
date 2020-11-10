from django.http import HttpResponse
from django.contrib.auth import authenticate, login
from django.contrib.auth.models import User
from django.contrib.auth.decorators import login_required
from django.db.utils import IntegrityError
from django.shortcuts import get_object_or_404, redirect, render
from unittest import skip
from .forms import LoginForm, AddClientForm, ClientLoginForm
from .models import Client
from random import choice

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
    return render(request, 'panel/clients.html', {"clients":clients})


@login_required
def add_client_screen(request):

    if request.method == 'POST':
        request.POST._mutable = True
        request.POST['pin'] = pin_generate()
        form = AddClientForm(data=request.POST)
        if form.is_valid():
            user = User.objects.get(username=request.user)
            try:
                form.save(user=user)
                form = AddClientForm()
                return render(request, 'panel/add_client.html', {'form': form, 'created_name':request.POST.get('name','')})
            except IntegrityError:
                # TODO: Tą walidację przenieś do formularza
                form.add_error(None, 'Klient o podanym numerze telefonu już istnieje')
                return render(request, 'panel/add_client.html', {'form': form})
    else:
        form = AddClientForm()
    return render(request, 'panel/add_client.html', {'form': form})

@login_required
def remove_client_screen(request, client_id):
    #TODO: Dodaj potwierdzenie usunięcia
    user = User.objects.get(username=request.user)
    Client.objects.filter(id=client_id,user=user).delete()
    return redirect(clients_screen)


    # TODO: Dodaj walidację - czy usuwany klient na pewno nalezy do tego uzytkownika
    # TODO: Wymuś potwierdzenie usunięcia

@login_required
def panel_screen(request):
    return render(request, 'panel/panel.html', {})
@login_required
def shedule_screen(request):
    return render(request, 'panel/shedule.html', {})


@login_required
def settings_screen(request):
    return render(request, 'panel/settings.html', {})



def client_panel(request, username):
    # TODO: Obsługa błędu 404
    try:
        user = User.objects.get(username__iexact=username)
        if user:
            form = ClientLoginForm()
            return render(request, 'client_panel/client_panel.html', {'form':form, 'user':user.username})
    except:
        # TODO: return 404
        return redirect(clients_screen)




# Funkcje pomocnicze
def pin_generate():
    """ Funkcja generuje losowy, 4-cyfrowy pin """
    pin = ''
    for i in range(0,4): pin+=choice('0123456789')
    return(pin)