from django.contrib.auth import views as auth_views
from django.urls import path
from . import views

urlpatterns = [
    path('<str:username>/', views.ClientAppLogin.as_view(), name='client_app_login'),
    path('<str:username>/panel/', views.ClientAppDashboard.as_view(), name='client_app_dashboard'),
    path('<str:username>/logout/', views.ClientAppLogout.as_view(), name='client_app_logout'),
    path('<str:username>/nowa_wizyta/<int:service_id>/', views.ClientAppNewVisit.as_view(), name='client_app_new_visit'),
    path('<str:username>/nowa_wizyta/<int:service_id>/<int:year>/<int:week>/', views.ClientAppNewVisit.as_view(), name='client_app_new_visit'),
    path('<str:username>/nowa_wizyta/<int:service_id>/<int:year>/<int:month>/<int:day>/<int:hour>/<int:minute>/', views.ClientAppConfirmVisit.as_view(), name='client_app_confirm_visit'),
    path('<str:username>/odwolaj/<int:visit_id>/', views.ClientAppCancelVisit.as_view(), name='client_app_cancel_visit'),
]