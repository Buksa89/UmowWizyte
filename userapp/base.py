from datetime import date, datetime, time, timedelta
from django.shortcuts import get_object_or_404
from django.urls import reverse
from django.utils import timezone

from random import choice

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



class ClientAddVisitSchedule(Schedule):
    def __init__(self, user, year, week, service):
        self.service = service
        self.user = user
        self.days = self.get_dates_from_week(year, week)
        self.title = self.service.name


    def get_navigation_dates(self):
        prev = self.days[0] - timedelta(days=7)
        next = self.days[0] + timedelta(days=7)
        return prev, next

    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        return reverse('client_app_new_visit', args=[self.user.username, self.service.id, year, week])

    def get_time_range(self):

        times = []

        for n, day in enumerate(self.days):
            name = day.strftime("%A").lower()

            start = self.work_time.__dict__['start_'+name]
            end = datetime.combine(date.min, start) + self.work_time.__dict__['duration_'+name]
            if end != datetime(1,1,2,0,0,0):
                end = end.time()
            else:
                end = time.max

            if start < end:
                times.append(start)
                times.append(end)

        if times:
            start = min(times)
            if max(times) != time.max:
                duration = datetime.combine(date.min, max(times)) - datetime.combine(date.min, start)
            else:
                duration = datetime(1,1,2,0,0,0) - datetime.combine(date.min, start)

            time_range = list_of_times_generate(start, duration)
            if time_range[-1] == time(0,0,0): time_range[-1] = time.max

        else:
            time_range = []

        return time_range

    def get_intervals(self):
        visit_intervals = self.get_visit_intervals()
        time_off_intervals = self.get_off_intervals()
        past_interval = self.get_past_intervals()
        available_intervals = []

        for d, day in enumerate(self.days):
            available_interval = interval.closedopen(not_naive(datetime.combine(day, time.min)),
                                                     not_naive(datetime.combine(day, time.max)))
            time_off_intervals[d] |= past_interval[d]
            time_off_intervals[d] |= visit_intervals[d]

            available_interval -= time_off_intervals[d]

            available_intervals.append(available_interval)

        return time_off_intervals, available_intervals


    def generate_time_off_content(self, d, time_off):
        html_code = ''
        if not time_off.empty:
            start_row = time_off.lower.strftime("%H%M")
            end_row = time_off.upper.strftime("%H%M")
            html_code += f'<div class="time-past day-{d + 1}" style="grid-column: day-{d + 1}; grid-row: time-{start_row} / time-{end_row};">' \
                         f'</div>'
            if time_off.upper >= not_naive(datetime.now()) and time_off.upper.date() == date.today():
                html_code += f'<div class="line-off day-{d + 1}" style="grid-column: day-{d + 1}; grid-row: time-{end_row};">' \
                             f'</div>'
        return html_code

    def schedule_content(self):
        html_code = ''
        time_off_intervals, available_intervals = self.get_intervals()

        for d, day in enumerate(self.days):
            for times_off in time_off_intervals[d]:
                html_code += self.generate_time_off_content(d, times_off)

            for available_time in available_intervals[d]:
                html_code += self.generate_available_time_content(d, day, available_time)

        return html_code

    def get_visit_url(self, date_tim):
        """ Method return link for choose visit by clients """
        return reverse('client_app_confirm_visit', args=[self.user.username, self.service.id, date_tim.year,
                                                         date_tim.month, date_tim.day, date_tim.hour, date_tim.minute])

    #TODO: cos z wyswietlaniem czasow nie tak
    def generate_available_time_content(self, d, day, available_time):
        simple_duration = int(self.service.duration / timedelta(minutes=15))
        html_code =''
        if not available_time.empty:
            field_number_base = (d * len(self.time_range)) - d
            for n, current_time in enumerate(self.time_range):
                field_number = field_number_base + n
                current_time = not_naive(datetime.combine(day, current_time))
                if current_time in available_time:

                    start_row = current_time.strftime("%H%M")
                    end_row = (current_time+timedelta(minutes=15)).strftime("%H%M")
                    if end_row == ("2400"): end_row = "2359"
                    #TODO: Sprawdzanie kolejnego dnia i blokowanie pola jesli wizyta sie nie wcisnie
                    duration_hours = self.service.duration.seconds//3600
                    duration_minutes = (self.service.duration.seconds//60)%60
                    visit_url = reverse('client_app_confirm_visit', args=[self.user.username, self.service.id,
                                                                          current_time.year, current_time.month,
                                                                         current_time.day, current_time.hour,
                                                                          current_time.minute])
                    # TODO Do refaktoryzacji  - chyba uda się połączyć z if current_time in available_time:
                    available = True
                    for step in range (1, simple_duration):
                        try:
                            next_time = not_naive(datetime.combine(day, self.time_range[n+step]))
                            if next_time not in available_time:
                                available = False
                                break
                        except:
                            available=False
                            break

                    if available:
                        html_code += f'<a id="{field_number}" href="{visit_url}" ' \
                                     f'class="available day-{d + 1}" style="grid-column: day-{d + 1}; ' \
                                     f'grid-row: time-{start_row}" ' \
                                     f'onmouseover="on_hover({field_number}, {simple_duration})" ' \
                                     f'onmouseout="out_hover({field_number}, {simple_duration})">' \
                                     f'</a>'
                    else:
                        html_code += f'<div id="{field_number}" href="{visit_url}" ' \
                                     f'class="available day-{d + 1}" style="grid-column: day-{d + 1}; ' \
                                     f'grid-row: time-{start_row}">' \
                                     f'</div>'

        return html_code

class UserAddVisitSchedule(Schedule):

    def __init__(self, user, year, week, client, service, duration):
        self.title= f'{client.name} {client.surname} - {service.name}'
        self.client = client
        self.service = service
        self.event_duration = duration
        self.user = user
        self.days = self.get_dates_from_week(year, week)
        self.time_range_type = 'full'
        self.visible_visits = True
        self.available_time = 'no_visits'

        self.prepare_data()

    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        hours = self.event_duration.seconds//3600
        minutes = (self.event_duration.seconds//60)%60
        return reverse('dashboard_new_visit_2', args=[self.client.id, self.service.id, hours, minutes, year, week])

    def get_available_url(self, time_, day):

        hours = self.event_duration.seconds//3600
        minutes = (self.event_duration.seconds//60)%60

        return reverse('dashboard_new_visit_3', args=[self.client.id, self.service.id, hours, minutes, day.year, day.month, day.day, time_.hour, time_.minute])

class UserLockTimeSchedule(Schedule):
    def __init__(self, user, year, week):
        self.user = user
        self.title= f'Wolne od:'
        self.days = self.get_dates_from_week(year, week)
        self.time_range_type = 'full'
        self.visible_visits = True
        self.available_time = 'all'
        self.event_duration = timedelta(minutes=15)


        self.prepare_data()

    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        return reverse('dashboard_lock_time_1', args=[year, week])

    def get_available_url(self, time_, day):

        return reverse('dashboard')


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



"""    def __init__(self, user, year, month, day):
        self.user = user
        day = datetime(year, month, day, 0, 0, 0)
        self.days = [day.date(), (day+timedelta(days=1)).date()]
        self.title = 'Terminarz'
        self.columns = [[1,4],[5,7]]

    def get_navigation_dates(self):
        prev = self.days[0] - timedelta(days=1)
        next = self.days[0] + timedelta(days=1)
        return prev, next

    def get_navigation_url(self, date):
        return reverse('dashboard', args=[date.year, date.month, date.day])

    def date_header(self):
        return ''

    def days_header(self):
        html_code = ''
        html_code += f'<span class="day-slot menu-line-strong" aria-hidden="true" style="grid-column: times / day-7; grid-row: days;"></span>'

        for n, day in enumerate(self.days):

            day_name = DAYS_OF_WEEK[day.weekday()]

            relative_day =''
            if day == date.today(): relative_day = ' (dzisiaj)'
            elif day == date.today() - timedelta(1): relative_day = ' (wczoraj)'
            elif day == date.today() + timedelta(1): relative_day = ' (jutro)'

            span_classes = ''
            if is_holiday(day) or day.weekday() == 6:
                span_classes += (' holiday')

            html_code += f'<span class="day-slot{span_classes}" aria-hidden="true" style="grid-column: day-{self.columns[n][0]}/day-{self.columns[n][1]}; grid-row: days;">{day_name}{relative_day}</span>'

        return html_code

    def generate_visit_content(self, d, visit_number, visit):
        start_row = visit.start.strftime("%H%M")
        end_row = visit.end.strftime("%H%M")
        html_code = f'<div class="session session-{visit_number} day-{d+1}" style="grid-column: day-{self.columns[d][0]}/day-{self.columns[d][1]}; grid-row: time-{start_row} / time-{end_row};">' \
                     f'<h3 class="session-title"><a href="{visit.get_display_url()}">{visit.name}</a></h3>' \
                     f'<p><a href="#">{visit.client.name}<br />' \
                     f'{visit.client.surname}<br /></a>' \
                     f'<a href="tel:{visit.client.phone_number}">{visit.client.phone_number}</a></p>' \
                     f'<p>{visit.description}</p>' \
                     f'</div>'
        return html_code

    def generate_time_off_content(self, d, times_off):
        html_code = ''
        if not times_off.empty:
            start_row = times_off.lower.strftime("%H%M")
            end_row = times_off.upper.strftime("%H%M")
            html_code += f'<div class="time-off day-{d + 1}" style="grid-column: day-{self.columns[d][0]}/day-{self.columns[d][1]}; grid-row: time-{start_row} / time-{end_row};">' \
                         f'</div>'
        return html_code

    def generate_past_content(self, d, past):
        html_code = ''
        if not past.empty:
            start_row = past.lower.strftime("%H%M")
            end_row = past.upper.strftime("%H%M")
            html_code += f'<div class="time-past day-{d + 1}" style="grid-column: day-{self.columns[d][0]}/day-{self.columns[d][1]}; grid-row: time-{start_row} / time-{end_row};">' \
                         f'</div>'
            if past.upper >= not_naive(datetime.now()) and past.upper.date() == date.today():
                html_code += f'<div class="line-off day-{d + 1}" style="grid-column: day-{self.columns[d][0]}/day-{self.columns[d][1]}; grid-row: time-{end_row};">' \
                             f'</div>'
        return html_code"""