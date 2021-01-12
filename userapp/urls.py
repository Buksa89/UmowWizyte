from django.contrib.auth import views as auth_views
from django.urls import path
from . import views

urlpatterns = [
    path('', views.Welcome.as_view(), name='welcome'),
    path('register/', views.Register.as_view(), name='register'),
    path('login/', views.Login.as_view(), name='login'),
    path('logout/', auth_views.LogoutView.as_view(), name='logout'),

    path('password_change/', auth_views.PasswordChangeView.as_view(), name='password_change'),
    path('password_change/done/', auth_views.PasswordChangeDoneView.as_view(), name='password_change_done'),
    path('password_reset/', auth_views.PasswordResetView.as_view(), name='password_reset'),
    path('password_reset/done/', auth_views.PasswordResetDoneView.as_view(), name='password_reset_done'),
    path('reset/<uidb64>/<token>/', auth_views.PasswordResetConfirmView.as_view(), name='password_reset_confirm'),
    path('reset/done/', views.PasswordResetComplete.as_view(), name='password_reset_complete'),








    path('panel/', views.Dashboard.as_view(), name='dashboard'),
    path('panel/<int:year>/<int:month>/<int:day>/', views.Dashboard.as_view(), name='dashboard'),
    path('panel/wizyta/<int:visit_id>/', views.DashboardVisit.as_view(), name='dashboard_visit'),
    path('panel/wizyta/<int:visit_id>/odrzuc/', views.DashboardVisitReject.as_view(), name='dashboard_visit_reject'),
    path('panel/wizyta/<int:visit_id>/potwierdz/', views.DashboardVisitConfirm.as_view(), name='dashboard_visit_confirm'),
    path('panel/wizyta/<int:visit_id>/odwolaj/', views.DashboardVisitCancel.as_view(), name='dashboard_visit_cancel'),
    path('panel/zablokuj_terminy/', views.DashboardLockTime.as_view(), name='dashboard_lock_time'),
    path('panel/zablokuj_terminy/<int:year>/<int:week>/', views.DashboardLockTime.as_view(), name='dashboard_lock_time'),
    path('panel/nowa_wizyta/<int:client_id>/<int:service_id>/<int:hours>/<int:minutes>/', views.DashboardNewVisit.as_view(), name='dashboard_new_visit'),
    path('panel/nowa_wizyta/<int:client_id>/<int:service_id>/<int:hours>/<int:minutes>/<int:year>/<int:week>/', views.DashboardNewVisit.as_view(), name='dashboard_new_visit'),
    path('panel/nowa_wizyta/<int:client_id>/<int:service_id>/<int:hours>/<int:minutes>/<int:year>/<int:month>/<int:day>/<int:hour>/<int:minute>/', views.DashboardConfirmVisit.as_view(), name='dashboard_confirm_visit'),

    path('terminarz/', views.DashboardSchedule.as_view(), name='dashboard_schedule'),
    path('terminarz/<int:year>/<int:week>/', views.DashboardSchedule.as_view(), name='dashboard_schedule'),

    path('klienci/', views.dashboard_clients, name='dashboard_clients'),
    path('klienci/nowy/', views.DashboardClientsAdd.as_view(), name='dashboard_clients_add'),
    path('klienci/usun/<int:client_id>/', views.dashboard_clients_remove, name='dashboard_clients_remove'),
    path('klienci/<int:client_id>/', views.DashboardClientsEdit.as_view(), name='dashboard_clients_edit'),

    path('settings/', views.SettingsServices.as_view(), name='settings'),
    path('settings/services/', views.SettingsServices.as_view(), name='settings_services'),
    path('settings/services/remove/<int:service_id>/', views.SettingsServiceRemove.as_view(), name='settings_services_remove'),
    path('settings/services/lock/<int:service_id>/', views.SettingsServiceLock.as_view(), name='settings_services_lock'),
    path('settings/work_time/', views.SettingsWorkTime.as_view(), name='settings_work_time'),

    path('settings/account/', views.SettingsAccount.as_view(), name='settings_account'),
    path('settings/contact/', views.SettingsContact.as_view(), name='settings_contact'),

]