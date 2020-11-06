from django.shortcuts import render
from unittest import skip


def welcome_screen(request):
    return render(request, 'welcome.html', {})


def login_screen(request):
    return render(request, 'login.html', {})
