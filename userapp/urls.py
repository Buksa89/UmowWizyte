from django.contrib.auth import views as auth_views
from django.urls import path
from . import views

urlpatterns = [
    path('', views.welcome_screen, name='welcome_screen'),

    path('login/', views.login_screen, name='login_screen'),
    path('logout/', auth_views.LogoutView.as_view(), name='logout_screen'),

    path('panel/', views.Dashboard.as_view(), name='dashboard'),
    path('panel/odrzuc_wizyte/<int:visit_id>/', views.DashboardVisitReject.as_view(), name='dashboard_visit_reject'),
    path('panel/potwierdz_wizyte/<int:visit_id>/', views.DashboardVisitConfirm.as_view(), name='dashboard_visit_confirm'),

    path('terminarz/', views.DashboardSchedule.as_view(), name='dashboard_schedule'),
    path('terminarz/<int:year>/<int:month>', views.DashboardSchedule.as_view(), name='dashboard_schedule'),
    path('terminarz/<int:year>/<int:month>/<int:day>', views.DashboardSchedule.as_view(), name='dashboard_schedule'),

    path('klienci/', views.dashboard_clients, name='dashboard_clients'),
    path('klienci/nowy/', views.DashboardClientsAdd.as_view(), name='dashboard_clients_add'),
    path('klienci/usun/<int:client_id>/', views.dashboard_clients_remove, name='dashboard_clients_remove'),

    path('ustawienia/', views.DashboardSettings.as_view(), name='dashboard_settings'),
    path('ustawienia/usun_usluge/<int:service_id>/', views.dashboard_settings_service_remove, name='dashboard_settings_service_remove'),
    path('ustawienia/blokada_uslugi/<int:service_id>/', views.DashboardSettingsServiceLock.as_view(), name='dashboard_settings_service_lock'),

]