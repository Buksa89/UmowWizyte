from django.contrib.auth import views as auth_views
from django.urls import path
from . import views

urlpatterns = [
    path('', views.welcome_screen, name='welcome_screen'),

    path('login/', views.login_screen, name='login_screen'),
    path('logout/', auth_views.LogoutView.as_view(), name='logout_screen'),

    path('panel/', views.panel_screen, name='panel_screen'),
    path('terminarz/', views.shedule_screen, name='shedule_screen'),
    path('klienci/', views.clients_screen, name='clients_screen'),
    path('ustawienia/', views.settings_screen, name='settings_screen'),

    path('<str:username>/', views.client_panel, name='client_panel'),

]