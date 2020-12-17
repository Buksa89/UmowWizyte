from datetime import date, datetime, time, timedelta
from django.contrib.auth.models import User
from django.db.models import Q
from django.shortcuts import get_object_or_404
from django.urls import reverse
from django.utils import timezone

import holidays
import portion as interval
from random import choice

from userapp.models import Client, Service, Visit, WorkTime

DAYS_FOR_CODE = ['monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday']
DAYS_OF_WEEK = ['Poniedziałek', 'Wtorek', 'Środa', 'Czwartek', 'Piątek', 'Sobota', 'Niedziela']
DAYS_OF_WEEK_SHORT = ['Pn','Wt','Śr','Cz','Pt','So','Nd']
MONTHS = ['Styczeń', 'Luty', 'Marzec', 'Kwiecień', 'Maj', 'Czerwiec', 'Lipiec', 'Sierpień', 'Wrzesień', 'Październik', 'Listopad', 'Grudzień']

def is_client_authenticated(request, username):
    """ Check client is logged in. Equivalent of 'user.is_authenticated' """
    if not request.session.get('client_authorized'): return False
    if request.session.get('client_authorized')['user'].lower() != username.lower(): return False
    return True

def is_holiday(date):
    """ Function get date and return True if it is holiday """
    if date in holidays.Poland(): return True

def is_free_day(date, work_time):
    """ Function get date and return True if it is not working day """
    if not work_time.__dict__[date.strftime("%A").lower()]: return True

def pin_generate():
    """ Generator 4-digits pin number for client """
    pin = ''
    for i in range(0,4): pin += choice('0123456789')
    return(pin)

def list_of_times_generate(start, duration, step=15):
    """ Generate list of datetime time objects from start to end with default step = 15 min. """
    list = []
    flag_time = datetime.combine(date.min, start)
    end = datetime.combine(date.min, start) + duration
    while flag_time <= end:
        list.append(flag_time.time())
        flag_time += timedelta(minutes=step)
    return list

def time_options(duration=timedelta(hours=8), sec=False):
    """ Generating list of times from 0 to hours (max:24). Step = 15min"""
    start = time(0, 0, 0)
    list = list_of_times_generate(start, duration)
    choices = []
    for n, hour in enumerate(list):
        value = hour
        
        if hour == time(0, 0, 0) and n == len(list)-1:
            label = "24:00"
        else:
            label = hour.strftime("%H:%M")

        if sec == False: value = label

        choices.append([value, label])
    return choices

def not_naive(dat):
    return timezone.make_aware(dat, timezone.get_default_timezone())



class Schedule:

    def __init__(self, user, year, week):
        self.title= 'Terminarz'
        self.user = user
        self.days = self.get_dates_from_week(year, week)


    #HTML
    def display(self):
        self.work_time = get_object_or_404(WorkTime, user=self.user)
        self.visits = self.get_visits()
        self.time_range = self.get_time_range()
        self.navigation = self.generate_navigation()
        self.id = 0

        html_code = '<div class="schedule-container">'
        html_code += self.title_header()
        html_code += '<div class="schedule" aria-labelledby="schedule-heading">'
        html_code += self.date_header()
        html_code += self.days_header()
        html_code += self.get_time_column_html()
        html_code += self.schedule_content()
        html_code += '</div></div>'

        return (html_code)

    def title_header(self):
        """ Method generates header of shedule with title and navigation """

        html_code = f'<div class="header"><ul>'
        if self.navigation:
            html_code += f'<a href="{self.navigation["prev"]}"><li class="prev">&#10094;</li></a>' \
                         f'<a href="{self.navigation["next"]}"><li class="next">&#10095;</li></a>'
        if self.title:
            html_code += f'<li>{self.title}</li>'
        html_code += f'</ul></div>'

        return html_code

    def days_header(self):
        """ Method generates header of schedule with days """
        html_code = ''

        html_code += f'<span class="day-slot" aria-hidden="true" style="grid-column: times; grid-row: days;"></span>'
        html_code += f'<span class="day-slot menu-line-strong" aria-hidden="true" style="grid-column: times / day-7; grid-row: days;"></span>'
        for n, day in enumerate(self.days):
            span_classes = ''
            span_content = DAYS_OF_WEEK_SHORT[n]
            if is_holiday(day) or day.weekday()==6:
                span_classes +=(' holiday')
            if day == date.today():
                span_classes +=(' today')
                span_content = f'<span>{span_content}</span>'

            html_code += f'<span class="day-slot{span_classes}" aria-hidden="true" style="grid-column: day-{n+1}; grid-row: days;">{span_content}</span>'

        return html_code

    def date_header(self):
        """ Method generates header of schedule with date """
        # get numbers of month appearance in days of current week
        html_code = ''
        years_and_months_dict = self.get_years_and_months(self.days)

        html_code += f'<span class="month-slot" aria-hidden="true" style="grid-column: times; grid-row: months;"></span>'
        end = 0
        for n, data in enumerate(years_and_months_dict):
            start = end + 1
            html_code += f'<span class="month-slot" aria-hidden="true" style="grid-column: day-{start}-start / '
            end = start + data['count'] - 1
            html_code += f'day-{end}-end; grid-row: months;">{data["year"]}<br/>{MONTHS[data["month"]-1]}</span>'

        html_code += f'<span class="day-number-slot" aria-hidden="true" style="grid-column: times; grid-row: day-number;"></span>'
        for n, day in enumerate(self.days):
            html_code += f'<span class="day-number-slot" aria-hidden="true" style="grid-column: day-{n+1}; grid-row: day-number;">{day.day}</span>'

        return html_code

    def get_time_column_html(self):
        """ Generate html for time column """
        html_code = ''
        for n, hour in enumerate(self.time_range):
            line_class = 'time-line'
            time_class = ''
            hour_css = hour.strftime('%H%M')
            if hour.minute == 15 or hour.minute==45:
                hour_display = ''
            else:
                hour_display = hour.strftime('%H:%M')
                if hour.minute == 00 or hour.minute == 59:
                    time_class = ' strong'
                    line_class = 'time-line-strong'
                    if n==len(self.time_range)-1 and hour == time.max:
                        hour_css = '2359'
                        hour_display = '24:00'

            html_code += f'<div class="{line_class}" style="grid-column: times / day-7; grid-row: time-{hour_css}"></div>'
            html_code += f'<h2 class="time-slot{time_class}" style="grid-row: time-{hour_css};"><span>{hour_display}</span></h2>'

        return html_code

    # Data

    def get_dates_from_week(self, year, week):
        """ Method gets year and week, and return 7 dates - days of this week """

        days = []
        for i in range(1, 8):
            days.append(date.fromisocalendar(year, week, i))
        return days

    def get_years_and_months(self, dates):
        """ Method generates list of months and years of current week. List is generated with dictionaries:
        {month, year, apperances}. Every week exist in one or two months. Dict gives information, how many days in week
        exist in this month, and how many in other. """

        dicts = []

        for date_ in dates:
            exist_in_dict = False

            for dict in dicts:
                if date_.month == dict['month']:
                    dict['count'] += 1
                    exist_in_dict = True
                    break

            if not exist_in_dict:
                dicts.append({'month': date_.month, 'year': date_.year, 'count': 1})

        return dicts

    #TODO: Czy na pewno tu powinno być datetime? czy może time?
    def load_visits(self):
        """ Method load visits for days from database. If one visit is in two days, is cutted
        in time 00:00 and exist in both days"""
        visits = []
        for day in self.days:
            day_visits = Visit.objects.filter(
                Q(user=self.user, start__year=day.year, start__month=day.month,  start__day=day.day) |
                Q(user=self.user, end__year=day.year, end__month=day.month, end__day=day.day))

            for visit in day_visits:
                if visit.start.date() < day:
                    print("sPrawdz load_visit")
                    visit.start = datetime.combine(day, time.min)
                if visit.end.date() > day:
                    print("sPrawdz load_visit")
                    visit.end = datetime.combine(day, time.max)
            visits.append(day_visits)

        return visits

    def generate_navigation(self):
        """ Method gets dates and return urls for shedule navigation """
        navigation = {}
        dates = {}
        dates['prev'], dates['next'] = self.get_navigation_dates()

        for key, date in dates.items():
            navigation[key] = self.get_navigation_url(date)
        return navigation

    def get_visit_intervals(self):
        """ Method prepare time intervals of visits to display correct fields in schedule """
        intervals = []
        for d, day in enumerate(self.days):
            day_intervals = interval.empty()
            for visit in self.visits[d]:
                day_intervals |= interval.closedopen(visit.start, visit.end)

            intervals.append(day_intervals)

        return intervals

    def get_off_intervals(self):
        """ Method prepare time intervals of not-working time to display correct fields in schedule """
        off_intervals = []
        for d, day in enumerate(self.days):
            name = day.strftime("%A").lower()
            start_work = not_naive(datetime.combine(day, self.work_time.__dict__['start_'+name]))
            end_work = start_work + self.work_time.__dict__['duration_'+name]
            interval_1 = interval.openclosed(not_naive(datetime.combine(day, time.min)), start_work)
            interval_2 = interval.openclosed(end_work, not_naive(datetime.combine(day, time.max)))
            intervals = interval_1 | interval_2

            off_intervals.append(intervals)

        return off_intervals

    def get_past_intervals(self):
        """ Method prepare time intervals of past days and times to display correct fields in schedule """
        past_intervals = []
        for d, day in enumerate(self.days):
            intervals = interval.empty()
            if day < date.today(): intervals = interval.openclosed(not_naive(datetime.combine(day, time.min)), not_naive(datetime.combine(day, time.max)))
            if day == date.today():
                rounded_time_now = datetime.now() + timedelta(minutes=15 - datetime.now().minute%15)
                intervals = interval.openclosed(not_naive(datetime.combine(day, time.min)), not_naive(rounded_time_now))

            past_intervals.append(intervals)

        return past_intervals

    def get_intervals(self):
        """ Prepare intervals to display """
        visit_intervals = self.get_visit_intervals()
        time_off_intervals = self.get_off_intervals()
        past_interval = self.get_past_intervals()
        available_intervals = []

        for d, day in enumerate(self.days):
            available_intervals.append([])
            time_off_intervals[d] -= past_interval[d]
            time_off_intervals[d] -= visit_intervals[d]
            past_interval[d] -= visit_intervals[d]

        return visit_intervals, time_off_intervals, past_interval, available_intervals

    def schedule_content(self):
        html_code = ''
        visit_intervals, time_off_intervals, past_interval, available_intervals = self.get_intervals()
        visit_number = 1
        for d, day in enumerate(self.days):
            for times_off in time_off_intervals[d]:
                html_code += self.generate_time_off_content(d, times_off)

            for past in past_interval[d]:
                html_code += self.generate_past_content(d, past)

            for available_time in available_intervals[d]:
                html_code += self.generate_available_time_content(d, day, available_time)

            for visit in self.visits[d]:
                html_code += self.generate_visit_content(d, visit_number, visit)
                visit_number += 1

        return html_code

    # To change in subclasses
    def get_navigation_dates(self):
        prev = self.days[0] - timedelta(days=7)
        next = self.days[0] + timedelta(days=7)
        return prev, next

    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        return reverse('dashboard_schedule', args=[year, week])

    def get_visits(self):
        """ Filter for visits to display """
        visits = self.load_visits()
        visits_filtered = []
        for days in visits:
            day_visits = []
            for visit in days:
                if (visit.is_available and visit.is_confirmed) or (not visit.is_available and not visit.is_confirmed):
                    day_visits.append(visit)
            visits_filtered.append(day_visits)

        return visits_filtered

    def get_time_range(self):
        """ Find all hours important to display on shedule. """

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

            for visit in self.visits[n]:
                start = visit.start.time()
                end = visit.end.time()
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

    def generate_past_content(self, d, past):
        html_code = ''
        if not past.empty:
            start_row = past.lower.strftime("%H%M")
            end_row = past.upper.strftime("%H%M")
            html_code += f'<div class="time-past day-{d + 1}" style="grid-column: day-{d + 1}; grid-row: time-{start_row} / time-{end_row};">' \
                         f'</div>'
        return html_code

    def generate_time_off_content(self, d, times_off):
        html_code = ''
        if not times_off.empty:
            start_row = times_off.lower.strftime("%H%M")
            end_row = times_off.upper.strftime("%H%M")
            html_code += f'<div class="time-off day-{d + 1}" style="grid-column: day-{d + 1}; grid-row: time-{start_row} / time-{end_row};">' \
                         f'</div>'
        return html_code
    #TODO: Niepotwierdzone niech wyswietlaja sie innym kolorem
    def generate_visit_content(self, d, visit_number, visit):
        start_row = visit.start.strftime("%H%M")
        end_row = visit.end.strftime("%H%M")
        html_code = f'<div class="session session-{visit_number} day-{d+1}" style="grid-column: day-{d+1}; grid-row: time-{start_row} / time-{end_row};">' \
                     f'<h3 class="session-title"><a href="{visit.get_display_url()}">{visit.name}</a></h3>' \
                     f'<p><a href="#">{visit.client.name}<br />' \
                     f'{visit.client.surname}<br /></a>' \
                     f'<a href="tel:{visit.client.phone_number}">{visit.client.phone_number}</a></p>' \
                     f'<p>{visit.description}</p>' \
                     f'</div>'
        return html_code

    def generate_available_time_content(self, d, day, available_time):
        html_code = ''
        return html_code

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
            start = datetime.combine(date.min, self.work_time.__dict__['start_' + name])
            end = start + self.work_time.__dict__['duration_' + name]
            if start < end:
                times.append(start)
                times.append(end)

        if times:
            start = min(times)
            duration = (max(times) - start)
            time_range = list_of_times_generate(start.time(), duration)
        else:
            time_range = []

        return time_range


    def generate_day_column(self, column_number, day, hours, statuses):
        html_code = ''
        simple_duration = int(self.service.duration / timedelta(minutes=15))
        display_day = True
        if day < (date.today() + timedelta(self.work_time.earliest_visit)) : display_day = False
        if day > (date.today() + timedelta(self.work_time.latest_visit)) : display_day = False
        if is_holiday(day) and not self.work_time.holidays: display_day = False
        for n, status in enumerate(statuses):
            hour_class = hours[n].strftime("%H-%M")
            if status == 'av' and display_day and hours[n] > not_naive(datetime.now()):
                flag = True
                # Check if next time quarters are free to put there new visitx
                for i in range(simple_duration):
                    try:
                        if statuses[i+n] == 'av':
                            continue
                    except:
                        pass
                    flag = False

                if flag:
                    html_code += f'<a href="{self.get_visit_url(hours[n])}" ' \
                                 f'onmouseover="on_hover({self.id}, {simple_duration})" ' \
                                 f'onmouseout="out_hover({self.id}, {simple_duration})">' \
                                 f'<li class="{hour_class} {column_number} avaliable" id="{self.id}">&nbsp;</li></a>'
                else:
                    html_code += f'<li class="{hour_class} {column_number} avaliable" id="{self.id}">&nbsp;</li>'
                self.id += 1
            else:
                html_code += f'<li class="{hour_class} {column_number} not-avaliable">&nbsp;</li>'
                self.id += 1

        return html_code


    def get_visit_url(self, date_tim):
        """ Method return link for choose visit by clients """
        return reverse('client_app_confirm_visit', args=[self.user.username, self.service.id, date_tim.year,
                                                         date_tim.month, date_tim.day, date_tim.hour, date_tim.minute])

class UserAddVisitSchedule(Schedule):

    def __init__(self, user, year, week, client, service, duration):
        self.title= f'{client.name} {client.surname} - {service.name}'
        self.client = client
        self.service = service
        self.duration = duration
        self.user = user
        self.days = self.get_dates_from_week(year, week)

    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        hours = self.duration.seconds//3600
        minutes = (self.duration.seconds//60)%60
        return reverse('dashboard_new_visit', args=[self.client.id, self.service.id, hours, minutes, year, week])

    def get_time_range(self):
        """ Find all hours important to display on shedule. """
        start = time.min
        duration = timedelta(hours=24)
        time_range = list_of_times_generate(start, duration)
        if time_range[-1] == time(0,0,0): time_range[-1] = time.max

        return time_range

    def get_intervals(self):
        visit_intervals = self.get_visit_intervals()
        time_off_intervals = self.get_off_intervals()
        past_interval = self.get_past_intervals()
        available_intervals = []

        for d, day in enumerate(self.days):
            time_off_intervals[d] -= past_interval[d]
            time_off_intervals[d] -= visit_intervals[d]
            past_interval[d] -= visit_intervals[d]
            available_interval = interval.closedopen(not_naive(datetime.combine(day, time.min)),
                                                     not_naive(datetime.combine(day, time.max)))

            available_interval -= past_interval[d]
            available_interval -= visit_intervals[d]
            available_intervals.append(available_interval)

        return visit_intervals, time_off_intervals, past_interval, available_intervals

    #TODO: podswietlanie wizyt za pozno o 15 mjin
    def generate_available_time_content(self, d, day, available_time):
        simple_duration = int(self.duration / timedelta(minutes=15))
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
                    duration_hours = self.duration.seconds//3600
                    duration_minutes = (self.duration.seconds//60)%60
                    visit_url = reverse('dashboard_confirm_visit', args=[self.client.id, self.service.id, duration_hours,
                                                                         duration_minutes, current_time.year, current_time.month,
                                                                         current_time.day, current_time.hour, current_time.minute])
                    # TODO Do refaktoryzacji  - chyba uda się połączyć z if current_time in available_time:
                    available = True
                    for step in range (1, simple_duration):
                        next_time = not_naive(datetime.combine(day, self.time_range[n+step]))
                        try:
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



class UserLockTimeSchedule(Schedule):
    def display(self, year, week):
        self.title = "Wybierz wolne godziny"
        self.days = self.get_dates_from_week(year, week)
        prev_date = self.days[0] - timedelta(days=7)
        next_date = self.days[0] + timedelta(days=7)
        self.navigation = self.generate_navigation(prev_date, next_date)
        self.available_dates = self.get_working_days(self.username)

        html_code = ''
        html_code += self.title_header()
        html_code += self.main_header()
        html_code += self.schedule_content()
        return (html_code)

    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        return reverse('dashboard_lock_time', args=[year, week])


class UserSchedule(Schedule):

    def __init__(self, user, year, week):
        self.title= 'Terminarz'
        self.user = user
        self.days = self.get_dates_from_week(year, week)


class UserTwoDaysSchedule(Schedule):
    def __init__(self, user, year, month, day):
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
        return html_code