from django.contrib import admin
from .models import Client, Service, TimeOff, UserSettings, Visit, WorkTime

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
                    'day_of_week',
                    'start',
                    'end',]

@admin.register(TimeOff)
class TimeOffAdmin(admin.ModelAdmin):
    list_display = ['user',
                    'start',
                    'end',]

@admin.register(UserSettings)
class UserSettingsAdmin(admin.ModelAdmin):
    list_display = ['user', 'site_url', 'site_name']