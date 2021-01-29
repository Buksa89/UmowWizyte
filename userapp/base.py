from datetime import date, datetime, time, timedelta
from django.shortcuts import get_object_or_404
from django.urls import reverse
from django.utils import timezone
import portion

from random import choice
from .variables import DAYS_OF_WEEK
from userapp.schedule import Schedule
from userapp.models import Client, Service, TimeOff, UserSettings, Visit, WorkTime



def get_user_from_url(url_key):
    settings = get_object_or_404(UserSettings, site_url=url_key)
    user = settings.user
    return user

def is_client_authenticated(request, username):
    """ Check client is logged in. Equivalent of 'user.is_authenticated' """
    if not request.session.get('client_authorized'): return False
    if request.session.get('client_authorized')['user'].lower() != username.lower(): return False
    return True

def pin_generate():
    """ Generator 4-digits pin number for client """
    pin = ''
    for i in range(0,4): pin += choice('0123456789')
    return(pin)

def td_to_string(td):
    hours = str(td.days*24 + td.seconds//3600).rjust(2,'0')
    minutes = str((td.seconds//60)%60).rjust(2,'0')
    string = f'{hours}:{minutes}'
    return string

def time_options(duration=8):
    """ Generating list of times from 0 to hours (max:24). Step = 15min"""
    choices = []
    flag_time = timedelta(seconds=0)
    while flag_time <= timedelta(hours=duration):
        #print(td_to_string(flag_time))
        value = flag_time
        label = td_to_string(flag_time)
        choices.append([value, label])
        flag_time += timedelta(minutes=15)

    return choices

def not_naive(dat):
    #return timezone.make_aware(dat, timezone.get_default_timezone())
    return dat


class UserAddVisitSchedule(Schedule):

    def __init__(self, user, year, week, client, service, duration):
        self.client = client
        self.service = service
        self.user = user
        self.title= f'{client.name} {client.surname} - {service.name}'
        self.days = self.get_dates_from_week(year, week)
        self.time_range = 'full' #full - all hours, #regular - only working hours, #extra - working hours+visits
        self.event_duration = duration
        self.prepare_data()

    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        hours = self.event_duration.seconds//3600
        minutes = (self.event_duration.seconds//60)%60
        return reverse('dashboard_new_visit_2', args=[self.client.id, self.service.id, hours, minutes, year, week])

    def get_available_url(self, time_):

        hours = self.event_duration.seconds//3600
        minutes = (self.event_duration.seconds//60)%60

        return reverse('dashboard_new_visit_3', args=[self.client.id, self.service.id, hours, minutes, time_.year, time_.month, time_.day, time_.hour, time_.minute])

    def generate_on_interval(self):
        interval = self.days_interval
        interval -= self.visits_intervals
        return interval

class UserLockTimeSchedule(Schedule):
    def __init__(self, user, year, week, start=False):

        self.user = user
        self.start = start
        if start:
            self.title = 'Wolne do:'
        else:
            self.title= f'Wolne od:'
        self.days = self.get_dates_from_week(year, week)
        self.time_range = 'full' #full - all hours, #regular - only working hours, #extra - working hours+visits
        self.event_duration = timedelta(minutes=15)

        self.prepare_data()

    def get_navigation_url(self, date):
        if self.start:
            cur_year = date.year
            week = date.isocalendar()[1]
            year = self.start.year
            month = self.start.month
            day = self.start.day
            hour = self.start.hour
            minute = self.start.minute
            return reverse('dashboard_lock_time_2', args=[cur_year, week, year, month, day, hour, minute])

        else:
            year = date.year
            week = date.isocalendar()[1]
            return reverse('dashboard_lock_time_1', args=[year, week])

    def get_available_url(self, time_):
        if self.start:
            start_year = self.start.year
            start_month = self.start.month
            start_day = self.start.day
            start_hour = self.start.hour
            start_minute = self.start.minute
            time_ += timedelta(minutes=15)
            end_year = time_.year
            end_month = time_.month
            end_day = time_.day
            end_hour = time_.hour
            end_minute = time_.minute
            return reverse('dashboard_lock_time_3', args=[start_year, start_month, start_day, start_hour, start_minute,
                                                          end_year, end_month, end_day, end_hour, end_minute])

        else:
            week = time_.isocalendar()[1]
            year = time_.year
            month = time_.month
            day = time_.day
            hour = time_.hour
            minute = time_.minute

            return reverse('dashboard_lock_time_2', args=[year, week, year, month, day, hour, minute])


    def generate_on_interval(self):
        interval = self.days_interval
        if self.start:
            interval -= portion.closedopen(-portion.inf, self.start)
        return interval

class ClientAddVisitSchedule(Schedule):

    def __init__(self, user, service, year, week):
        self.title= f'{service.name}'
        self.service = service
        self.user = user
        self.event_duration = service.duration
        self.url_key = get_object_or_404(UserSettings, user=user).site_url
        self.days = self.get_dates_from_week(year, week)
        self.time_range = 'regular' #full - all hours, #regular - only working hours, #extra - working hours+visits

        self.prepare_data()

    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        return reverse('client_new_visit', args=[self.url_key, self.service.id, year, week])

    def generate_on_interval(self):
        available_intervals = self.days_interval
        available_intervals -= self.off_interval
        available_intervals -= self.past_interval
        available_intervals -= self.visits_intervals

        return available_intervals

    def generate_off_interval(self):
        interval = self.days_interval
        interval -= self.work_interval
        interval |= self.past_interval
        interval |= self.time_off_interval
        interval |= self.visits_intervals

        return interval

    def generate_visit_content(self, d, visit):
        html_code = ''
        return html_code

    def get_available_url(self, time_):

        return reverse('client_confirm_visit', args=[self.url_key, self.service.id, time_.year, time_.month, time_.day, time_.hour, time_.minute])


class UserTwoDaysSchedule(Schedule):

    def __init__(self, user, year, month, day):
        self.title= 'Terminarz'
        self.user = user
        day = datetime(year, month, day, 0, 0, 0)
        self.days = [day.date(), (day+timedelta(days=1)).date(), (day+timedelta(days=2)).date()]
        self.time_range_type = 'visits'
        self.visible_visits = True
        self.available_time = False

        self.prepare_data()

    def __init__(self, user, year, month, day):
        self.title= 'Terminarz'
        self.user = user
        day = date(year, month, day)
        self.days = [day - timedelta(days=1), day, day+timedelta(days=1), day+timedelta(days=2)]
        self.time_range = 'extra' #full - all hours, #regular - only working hours, #extra - working hours+visits
        self.prepare_data()
        self.event_duration = timedelta(seconds=0)
        self.columns = [[1,4],[5,7]]

        self.prepare_data()

    def get_navigation_dates(self):
        prev = self.days[1] - timedelta(days=1)
        next = self.days[1] + timedelta(days=1)
        return prev, next

    def get_navigation_url(self, date):
        return reverse('dashboard', args=[date.year, date.month, date.day])

    def date_header(self):
        return ''

    def days_header(self):
        html_code = ''
        html_code += f'<span class="day-slot menu-line-strong" aria-hidden="true" style="grid-column: times / day-7; grid-row: days;"></span>'
        for n, day in enumerate(self.days[1:-1]):

            day_name = DAYS_OF_WEEK[day.weekday()]

            relative_day =''
            if day == date.today(): relative_day = ' (dzisiaj)'
            elif day == date.today() - timedelta(1): relative_day = ' (wczoraj)'
            elif day == date.today() + timedelta(1): relative_day = ' (jutro)'

            span_classes = ''
            if self.is_holiday(day) or day.weekday() == 6:
                span_classes += (' holiday')

            html_code += f'<span class="day-slot{span_classes}" aria-hidden="true" style="grid-column: day-{self.columns[n][0]}/day-{self.columns[n][1]}; grid-row: days;">{day_name}{relative_day}</span>'

        return html_code

    def time_off_content(self, d, times_off):
        html_code = ''
        if not times_off.empty:
            start_row = times_off.lower.strftime("%H%M")
            end_row = times_off.upper.strftime("%H%M")
            if end_row == "0000": end_row = "2400"

            html_code += f'<div class="time-off day-{d + 1}" style="grid-column: day-{self.columns[d][0]}/day-{self.columns[d][1]}; grid-row: time-{start_row} / time-{end_row};">' \
                         f'</div>'

        return html_code

    def generate_visit_content(self, d, visit):
        html_code = ''

        start_row = visit.start.strftime("%H%M")
        end_row = visit.end.strftime("%H%M")

        if end_row == '0000': end_row = '2400'
        color_light = ''

        if not visit.is_confirmed:
            color_light = ' color-light'

        client_url = reverse('clients_data', args=[visit.client.id])
        html_code = f'<div class="session day-{d+1}{color_light}" style="grid-column: day-{self.columns[d][0]}/day-{self.columns[d][1]}; grid-row: time-{start_row} / time-{end_row};">' \
                     f'<h3 class="session-title"><a href="{visit.get_display_url()}">{visit.name}</a></h3>' \
                     f'<p><a href="#">{visit.client.name}<br />' \
                     f'{visit.client.surname}<br /></a>' \
                     f'<a href="tel:{visit.client.phone_number}">{visit.client.phone_number}</a></p>' \
                     f'<p>{visit.description}</p>' \
                     f'</div>'
        return html_code

    def schedule_content(self):
        html_code = ''
        for d, day in enumerate(self.days[1:-1]):
            dt_day = datetime.combine(day, time.min)
            day_interval = portion.closedopen(dt_day + self.start_day, dt_day + self.end_day)
            day_off = self.off_interval & day_interval
            for off_interval in day_off:
                html_code += self.time_off_content(d, off_interval)

            for visit in self.visits_per_day[d]:
                html_code += self.generate_visit_content(d, visit)

            current_time = self.get_current_time()
            if current_time.date() == day:
                str_time = current_time.strftime('%H%M')
                html_code += f'<div class="line-off day-{d + 1}" style="grid-column: day-{self.columns[d][0]}/day-{self.columns[d][1]}; grid-row: time-{str_time};">' \
                             f'</div>'

        html_code += self.time_on_content()

        return html_code
