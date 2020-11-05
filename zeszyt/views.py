from django.shortcuts import render

def welcome_screen(request):
    return render(request, 'welcome.html', {})
