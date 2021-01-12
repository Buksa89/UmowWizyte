from django.contrib import admin
from .models import Client, Service, UserSettings, Visit, WorkTime

@admin.register(Client)
class ClientAdmin(admin.ModelAdmin):
    list_display = ['user', 'phone_number', 'email', 'name', 'surname', 'description', 'pin', 'is_active']

@admin.register(Service)
class ServiceAdmin(admin.ModelAdmin):
    list_display = ['user', 'name', 'duration', 'is_active']

@admin.register(Visit)
class VisitAdmin(admin.ModelAdmin):
    list_display = ['user', 'client', 'name', 'start', 'end', 'is_available', 'is_confirmed', 'description']

@admin.register(WorkTime)
class WorkTimeAdmin(admin.ModelAdmin):
    list_display = ['user',
                    'start_monday', 'duration_monday',
                    'start_tuesday', 'duration_tuesday',
                    'start_wednesday', 'duration_wednesday',
                    'start_thursday', 'duration_thursday',
                    'start_friday', 'duration_friday',
                    'start_saturday', 'duration_saturday',
                    'start_sunday', 'duration_sunday',
                    'holidays',
                    'earliest_visit', 'latest_visit']


@admin.register(UserSettings)
class UserSettingsAdmin(admin.ModelAdmin):
    list_display = ['user', 'site_url', 'site_name']