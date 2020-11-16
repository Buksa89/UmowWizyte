from django.contrib.auth import views as auth_views
from django.urls import path
from . import views

urlpatterns = [
    path('', views.welcome_screen, name='welcome_screen'),

    path('login/', views.login_screen, name='login_screen'),
    path('logout/', auth_views.LogoutView.as_view(), name='logout_screen'),

    path('panel/', views.dashboard, name='dashboard'),

    path('terminarz/', views.dashboard_schedule, name='dashboard_schedule'),
    path('terminarz/<int:year>/<int:month>', views.dashboard_schedule, name='dashboard_schedule'),
    path('terminarz/<int:year>/<int:month>/<int:day>', views.dashboard_schedule, name='dashboard_schedule'),

    path('klienci/', views.dashboard_clients, name='dashboard_clients'),
    path('klienci/nowy/', views.dashboard_clients_add, name='dashboard_clients_add'),
    path('klienci/usun/<int:client_id>/', views.dashboard_clients_remove, name='dashboard_clients_remove'),

    path('ustawienia/', views.dashboard_settings, name='dashboard_settings'),
    path('ustawienia/usun_usluge/<int:service_id>/', views.dashboard_settings_service_remove,
         name='dashboard_settings_service_remove'),

    path('<str:username>/', views.client_app, name='client_app'),
    path('<str:username>/logout/', views.client_logout, name='client_logout'),      #TODO: client_app_logout
    path('<str:username>/nowa_wizyta/', views.client_app_new_visit_step_1, name='client_app_new_visit_step_1'),

]