from django.http import HttpResponse
from django.contrib.auth import authenticate, login
from django.contrib.auth.decorators import login_required
from django.shortcuts import redirect, render
from unittest import skip
from .forms import LoginForm


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
def panel_screen(request):
    return render(request, 'panel/panel.html', {})
@login_required
def shedule_screen(request):
    return render(request, 'panel/shedule.html', {})
@login_required
def clients_screen(request):
    return render(request, 'panel/clients.html', {})
@login_required
def settings_screen(request):
    return render(request, 'panel/settings.html', {})