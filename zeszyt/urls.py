from django.contrib.auth import views as auth_views
from django.urls import path
from . import views

urlpatterns = [
    path('', views.welcome_screen, name='welcome_screen'),

    path('login/', views.login_screen, name='login_screen'),
    path('logout/', auth_views.LogoutView.as_view(), name='logout_screen'),

    path('panel/', views.dashboard, name='dashboard'),

    path('terminarz/', views.DashboardSchedule.as_view(), name='dashboard_schedule'),
    path('terminarz/<int:year>/<int:month>', views.DashboardSchedule.as_view(), name='dashboard_schedule'),
    path('terminarz/<int:year>/<int:month>/<int:day>', views.DashboardSchedule.as_view(), name='dashboard_schedule'),

    path('klienci/', views.dashboard_clients, name='dashboard_clients'),
    path('klienci/nowy/', views.DashboardClientsAdd.as_view(), name='dashboard_clients_add'),
    path('klienci/usun/<int:client_id>/', views.dashboard_clients_remove, name='dashboard_clients_remove'),

    path('ustawienia/', views.DashboardSettings.as_view(), name='dashboard_settings'),
    path('ustawienia/usun_usluge/<int:service_id>/', views.dashboard_settings_service_remove, name='dashboard_settings_service_remove'),

    path('<str:username>/', views.ClientAppLogin.as_view(), name='client_app_login'),
    path('<str:username>/panel/', views.ClientAppDashboard.as_view(), name='client_app_dashboard'),
    path('<str:username>/logout/', views.ClientAppLogout.as_view(), name='client_app_logout'),
    path('<str:username>/nowa_wizyta/<int:service_id>/', views.ClientAppNewVisit.as_view(), name='client_app_new_visit'),# TODO: Testy url
    path('<str:username>/nowa_wizyta/<int:service_id>/<int:year>/<int:week>/', views.ClientAppNewVisit.as_view(), name='client_app_new_visit'),# TODO: Testy url
    path('<str:username>/nowa_wizyta/<int:service_id>/<int:year>/<int:month>/<int:day>/<int:hour>/<int:minute>', views.ClientAppConfirmVisit.as_view(), name='client_app_confirm_visit'),# TODO: Testy url

    path('<str:username>/usun/<int:visit_id>/', views.ClientAppCancelVisit.as_view(), name='client_app_cancel_visit') # TODO: Testy url

]