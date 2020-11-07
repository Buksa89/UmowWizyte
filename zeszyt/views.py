from django.http import HttpResponse
from django.contrib.auth import authenticate, login
from django.contrib.auth.models import User
from django.contrib.auth.decorators import login_required
from django.shortcuts import get_object_or_404, redirect, render
from unittest import skip
from .forms import LoginForm, AddClientForm
from .models import Client
from random import randrange

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
                if user is not None:
                    if user.is_active:
                        login(request, user)
                        return redirect(panel_screen)
                    else:
                        return HttpResponse('Konto zablokowane')
                else:
                    return HttpResponse('Błędny login lub hasło')
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
    form = AddClientForm(data=request.POST)
    if form.is_valid():
        user = User.objects.get(username=request.user)
        form.save(user=user, pin=pin_generate(), next_pin=pin_generate())
        form = AddClientForm()
        return render(request, 'panel/add_client.html', {'form': form, 'created_name':request.POST.get('name','')})
    return render(request, 'panel/add_client.html', {'form': form})
    # TODO: Dodaj walidację formularza dodawania klienta
    # TODO: Dodaj walidacje duplikatów klientów

@login_required
def remove_client_screen(request, client_id):

    Client.objects.filter(id=client_id).delete()
    #return render(request, 'panel/remove_client.html', {})
    return redirect(clients_screen)

    # TODO: Obsługa błędu jeśli klient o takim id nie istnieje
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
    user = User.objects.get(username__iexact=username)
    if user:
        return render(request, 'client_panel/client_panel.html', {'username':user.username})
    # TODO: Obsługa błędu 404



# Funkcje pomocnicze
def pin_generate():
    """ Funkcja generuje losowy, 4-cyfrowy pin """
    pin = f'{randrange(1, 10**4):4}'
    return(pin)