from django.urls import path
from . import views

urlpatterns = [
    path('', views.welcome_screen, name='welcome_screen'),
]
