from django.contrib.auth import views as auth_views
from django.urls import path
from . import views

urlpatterns = [
    path('<str:url_key>/', views.ClientLogin.as_view(), name='client_login'),
    path('<str:url_key>/dashboard/', views.Dashboard.as_view(), name='client_dashboard'),
    path('<str:url_key>/logout/', views.ClientAppLogout.as_view(), name='client_logout'),
    path('<str:url_key>/new_visit/<int:service_id>/', views.NewVisit.as_view(), name='client_new_visit'),
    path('<str:url_key>/new_visit/<int:service_id>/<int:year>/<int:week>/', views.NewVisit.as_view(), name='client_new_visit'),
    path('<str:url_key>/new_visit/<int:service_id>/<int:year>/<int:month>/<int:day>/<int:hour>/<int:minute>/', views.ClientAppConfirmVisit.as_view(), name='client_confirm_visit'),
    path('<str:url_key>/cancel/<int:visit_id>/', views.ClientAppCancelVisit.as_view(), name='client_cancel_visit'),
]