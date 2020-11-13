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
    path('klienci/nowy/', views.add_client_screen, name='add_client_screen'),
    path('klienci/usun/<int:client_id>/', views.remove_client_screen, name='remove_client_screen'),

    path('ustawienia/', views.settings_screen, name='settings_screen'),
    path('panel/usun_usluge/<int:service_id>/', views.remove_service, name='remove_service'),

    path('<str:username>/', views.client_login, name='client_login'),
    path('<str:username>/panel/', views.client_panel, name='client_panel'),
    path('<str:username>/logout/', views.client_logout, name='client_logout'),

]