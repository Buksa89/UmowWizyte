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
        self.title= ''
        self.user = user
        self.days = self.get_dates_from_week(year, week)


    #HTML
    def display(self):
        self.work_time = get_object_or_404(WorkTime, user=self.user)
        self.visits = self.load_visits(self.days)
        self.time_range = self.get_time_range()
        self.navigation = self.generate_navigation()
        self.id = 0

        html_code = '<div class="schedule">'
        html_code += self.title_header()
        html_code += self.main_header()
        html_code += self.schedule_content()
        html_code += '</div>'

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

    def main_header(self):
        """ Method generates header of schedule with days, months and years for current week """

        html_code = '<div class="dates-header">'
        html_code += '<ul><li class="empty">&nbsp;</li>'

        # Year and month display
        # get numbers of month appearance in days of current week
        years_and_months_to_display_dict = self.get_years_and_months(self.days)

        for n, data in enumerate(years_and_months_to_display_dict):
            extra_class = ''

            # when is more than one month in current week, first day of next month should get extra css_style
            if len(years_and_months_to_display_dict) > 1 and n == len(years_and_months_to_display_dict)-1:
                extra_class = ' class="border-date"'

            # header is divided in 8 parts. First is empty. So every part is related to one day. If we have some days in
            # one month, and rest in second month, every month should get width: appearances * 12,5% of header
            html_code += f"<li style=\"width:{12.5*data['count']}%\"{extra_class}>" \
                         f"{data['year']}<br />" \
                         f"{MONTHS[data['month']-1]}" \
                         f"</li>"
        html_code += '</ul>'

        # Days display

        html_code += f'<ul class="days"><li class="empty">&nbsp;</li>'
        for n, day in enumerate(self.days):
            css_class = []
            if is_holiday(day) or n == 6: css_class.append('red')
            if datetime.today().date() == day: css_class.append('today')
            if n == years_and_months_to_display_dict[0]['count']: css_class.append('border-date')
            css_class = ' '.join(css_class)
            if css_class: css_class = f' class="{css_class}"'

            html_code += f'<li{css_class}><div>{day.day}' \
                         f'<br />{DAYS_OF_WEEK_SHORT[n]}</div></li>'
        html_code += f'</ul></div>'

        return html_code

    def schedule_content(self):

        html_code = f'<div>'
        html_code += f'<ul class="schedule-content"><li>'
        html_code += f'<ul class="hours">'
        html_code += self.get_time_column_html(self.time_range)
        html_code += f'</ul></li>'


        for d, day in enumerate(self.days):
            html_code += f'<li><ul class="visit" id="day-{d}">'
            html_code += self.prepare_day_column(day, self.visits[d], self.time_range)
            html_code += f'</ul></li>'

        html_code += f'</ul></div>'
        return html_code

    def get_time_column_html(self, working_hours):
        """ Generate html for time column """

        html_code = ''
        for n, hour in enumerate(working_hours):
            if hour != datetime(1,1,2,0,0,0):
                hour_display = hour.strftime("%H:%M")
            else:
                hour_display = "24:00"
            if n != len(working_hours) - 2:
                html_code += f'<li><span>{hour_display}</span></li>'
            else:
                html_code += f'<li><span>{hour_display}</span><br/><span class="last">{working_hours[-1].strftime("%H:%M")}</span></li>'
                break


        return html_code

    #stałe

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

    #TODO: Sprawdz czy wizyty są w zakresie 00:00, 24:00
    def load_visits(self, days):
        """ Method load visits for days from database. If one visit is in two days, is cutted
        in tim 00:00 and exist in both days"""
        visits = []
        for day in days:
            day_visits = Visit.objects.filter(
                Q(user=self.user, start__year=day.year, start__month=day.month,  start__day=day.day) |
                Q(user=self.user, end__year=day.year, end__month=day.month, end__day=day.day))

            for visit in day_visits:
                if visit.start.date() < day:
                    visit.start = datetime.min
                if visit.end.date() > day:
                    visit.end = datetime.combine(day + datetime.max)
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

    def visit_intervals(self, visits):
        visits_list = []
        intervals = []
        for visit in visits:
            visit_interval = interval.closedopen(visit.start, visit.end)
            visits_list.append(visit)
            intervals.append(visit_interval)
        return visits_list, intervals

    def hours_off_intervals(self, day):
        name = day.strftime("%A").lower()
        start_work = not_naive(datetime.combine(day, self.work_time.__dict__['start_'+name]))
        duration = self.work_time.__dict__['duration_'+name]
        end_work = start_work + duration

        off_intervals = []

        if day >= datetime.now().date():
            off_intervals.append(interval.closedopen(not_naive(datetime.combine(day, time.min)), start_work))
            off_intervals.append(interval.closedopen(end_work, not_naive(datetime.combine(day, time.max))))
        else:
            off_intervals.append(interval.closedopen(not_naive(datetime.combine(day, time.min)),
                                                     not_naive(datetime.combine(day, time.max))))

        return off_intervals

    # To change in subclasses
    def get_navigation_dates(self):
        prev = self.days[0] - timedelta(days=7)
        next = self.days[0] + timedelta(days=7)
        return prev, next

    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        return reverse('dashboard')


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

            times.append(start)
            times.append(end)

            for visit in self.visits[n]:
                start = visit.start.time()
                end = visit.end.time()
                times.append(start)
                times.append(end)

        start = min(times)
        duration = datetime.combine(date.min, max(times)) - datetime.combine(date.min, start)

        time_range = list_of_times_generate(start, duration)

        return time_range

    def prepare_day_column(self, day, visits, time_range):
        """ Method prepare variables for generating day colums and call mothod to do it. """
        html_code = ''

        visits_list, visits_intervals = self.visit_intervals(visits)
        hours_off_intervals = self.hours_off_intervals(day)
        time_status = []
        hours = []

        for hour in time_range[:-1]:
            hour = timezone.make_aware(datetime.combine(day, hour), timezone.get_default_timezone())
            hours.append(hour)

            for n, visit in enumerate(visits_intervals):
                if hour in visit:
                    # Time booked for visit.id
                    time_status.append(visits_list[n].id)
                    break
            else:
                for interval in hours_off_intervals:
                    if hour in interval:
                        # Ouf of working hours
                        time_status.append('off')
                        break
                else:
                    # This time is free to book
                    time_status.append('av')
                    continue

        html_code += self.generate_day_column(hours, time_status)

        return html_code

    def generate_day_column(self, hours, statuses):
        html_code = ''
        for n, status in enumerate(statuses):
            if status == 'av':
                html_code += f'<li>&nbsp;</li>'
            elif status == 'off':
                html_code += f'<li class="hour-off">&nbsp;</li>'
            else:
                visit = Visit.objects.get(user=self.user, id=status)
                html_code += f'<li>{visit.name}</li>'

        return html_code






    def get_term_url(self, term):
        """ Method return link for choose visit by clients """
        pass
        #return reverse('client_app_confirm_visit', args=[self.username, self.service.id,
        #                                                 term.year, term.month, term.day, term.hour, term.minute])







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
            times.append(start)
            times.append(end)

        start = min(times)
        duration = (max(times) - start)

        time_range = list_of_times_generate(start.time(), duration)

        return time_range

    def generate_day_column(self, hours, statuses):
        html_code = ''
        simple_duration = int(self.service.duration / timedelta(minutes=15))
        for n, status in enumerate(statuses):
            if status == 'av':
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
                                 f'<li class="avaliable" id="{self.id}">&nbsp;</li></a>'
                else:
                    html_code += f'<li class="avaliable" id="{self.id}">&nbsp;</li>'
                self.id += 1
            elif status == 'off':
                html_code += f'<li class="hour-off">&nbsp;</li>'
                self.id += 1
            else:
                visit = Visit.objects.get(user=self.user, id=status)
                html_code += f'<li class="not-avaliable">&nbsp;</li>'
                self.id += 1

        return html_code


    def get_visit_url(self, date_tim):
        """ Method return link for choose visit by clients """
        return reverse('client_app_confirm_visit', args=[self.user.username, self.service.id, date_tim.year,
                                                         date_tim.month, date_tim.day, date_tim.hour, date_tim.minute])








class UserAddVisitSchedule(Schedule):
    def display(self, client, service, year, week, duration):
        self.client = client
        self.service = service
        self.duration = duration
        self.title = f'{client.name} {client.surname} - {service.name}'
        self.days = self.get_dates_from_week(year, week)
        prev_date = self.days[0] - timedelta(days=7)
        next_date = self.days[0] + timedelta(days=7)
        self.navigation = self.generate_navigation(prev_date, next_date)
        self.available_dates = self.get_working_days(self.username)

        html_code = ''
        html_code += self.title_header()
        html_code += self.main_header()
        html_code += self.schedule_content()
        return html_code

    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        hours = self.duration.seconds//3600
        minutes = (self.duration.seconds//60)%60
        return reverse('dashboard_new_visit', args=[self.client.id, self.service.id, hours, minutes, year, week])


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
    pass
    """    def display(self, year, week):
        self.title = "Terminarz"
        self.days = self.get_dates_from_week(year, week)
        prev_date = self.days[0] - timedelta(days=7)
        next_date = self.days[0] + timedelta(days=7)
        self.navigation = self.generate_navigation(prev_date, next_date)
        self.available_dates = self.get_working_days(self.username)

        html_code = ''
        html_code += self.title_header()
        html_code += self.main_header()
        html_code += self.schedule_content()
        html_code += f'<a href="{reverse("dashboard_schedule", args=[datetime.today().year, datetime.today().month, datetime.today().day])}">aa</a>'
        return (html_code)"""

    """def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        return reverse('dashboard_schedule', args=[year, week])"""


class UserTwoDaysSchedule(Schedule):
    def __init__(self, user, year, month, day):
        self.user = user
        day = datetime(year, month, day, 0, 0, 0)
        self.days = [day.date(), (day+timedelta(days=1)).date()]
        self.title = 'Terminarz'

    def main_header(self):

        html_code = '<div class="dates-header">'
        html_code += '<ul><li class="empty">&nbsp;</li>'
        html_code += f'<li style="width:62.5%">Dziś</li>'
        html_code += f'<li style="width:25%">Jutro</li>'
        html_code += '</ul></div>'

        return html_code

    def get_navigation_dates(self):
        prev = self.days[0] - timedelta(days=1)
        next = self.days[0] + timedelta(days=1)
        return prev, next

    def get_navigation_url(self, date):
        return reverse('dashboard', args=[date.year, date.month, date.day])


class UserOneDaySchedule(Schedule):
    def __init__(self, user, year=datetime.now().year, month=datetime.now().month, day=datetime.now().day):
        self.user = user
        day = date(year, month, day)
        self.days = [day, day + timedelta(day=1)]
        self.title = self.service.name


#        self.title = "Dzien" #Data, nazwa dnia, czy dziś, czy święto
#        prev_date = datetime(year, month, day) - timedelta(days=1)
#        next_date = datetime(year, month, day) + timedelta(days=1)
#        self.days = [datetime(year, month, day)]
#        self.navigation = self.generate_navigation(prev_date, next_date)

        html_code = ''
#        html_code += self.title_header()
        #html_code += '<div class="dates-header" style="height:20px;"></div>'
        #html_code += self.schedule_content()
        return (html_code)

    def get_navigation_url(self, date):
        return reverse('dashboard_schedule', args=[date.year, date.month, date.day])


    def generate_day_column(self, day, time_list):
        """ Generate html for day column """
        html_code = ''
        for hour in time_list:
            html_code += f'<li>&nbsp;</li>'
        return html_code