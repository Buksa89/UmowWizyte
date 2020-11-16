from django.contrib.auth import views as auth_views
from django.urls import path
from . import views

urlpatterns = [
    path('', views.welcome_screen, name='welcome_screen'),

    path('login/', views.login_screen, name='login_screen'),
    path('logout/', auth_views.LogoutView.as_view(), name='logout_screen'),

    path('panel/', views.panel_screen, name='panel_screen'),
    path('terminarz/', views.schedule_screen, name='schedule_screen'),
    path('terminarz/<int:year>/<int:month>', views.schedule_screen, name='schedule_screen'),
    path('terminarz/<int:year>/<int:month>/<int:day>', views.schedule_screen, name='schedule_screen'),
    path('klienci/', views.clients_screen, name='clients_screen'),
    path('klienci/nowy/', views.add_client_screen, name='add_client_screen'),
    path('klienci/usun/<int:client_id>/', views.remove_client_screen, name='remove_client_screen'),

    path('ustawienia/', views.settings_screen, name='settings_screen'),
    path('panel/usun_usluge/<int:service_id>/', views.remove_service, name='remove_service'),

    path('<str:username>/', views.client_app, name='client_app'),
    path('<str:username>/logout/', views.client_logout, name='client_logout'),
    path('<str:username>/nowa_wizyta/', views.client_app_new_visit_step_1, name='client_app_new_visit_step_1'),

]