from datetime import date, datetime, time, timedelta
from django.contrib.auth.models import User
from django.db.models import Q
from django.shortcuts import get_object_or_404
from django.urls import reverse
from django.utils import timezone

import holidays
import portion as interval
from random import choice

from userapp.models import Service, Visit, WorkTime

DAYS_OF_WEEK = ['Poniedziałek', 'Wtorek', 'Środa', 'Czwartek', 'Piątek', 'Sobota', 'Niedziela']
DAYS_OF_WEEK_SHORT = ['Pn','Wt','Śr','Cz','Pt','So','Nd']
MONTHS = ['Styczeń', 'Luty', 'Marzec', 'Kwiecień', 'Maj', 'Czerwiec', 'Lipiec', 'Sierpień', 'Wrzesień', 'Październik', 'Listopad', 'Grudzień']

def is_client_authenticated(request, username):
    """ Check client is logged in. Equivalent of 'user.is_authenticated' """
    if not request.session.get('client_authorized'): return False
    if request.session.get('client_authorized')['user'].lower() != username.lower(): return False
    return True

def get_available_days_for_clients(username):
    """ Function get work_time of user and check what days his clients can choose for visit """
    user = get_object_or_404(User, username__iexact=username)
    work_time = get_object_or_404(WorkTime, user=user)
    available_date = []
    for days in range (work_time.earliest_visit, work_time.latest_visit+1):
        date = datetime.today() + timedelta(days=days)
        if not work_time.holidays and is_holiday(date): continue
        day_name = date.strftime("%A").lower()
        if not work_time.__dict__[day_name]: continue
        available_date.append(date.date())
    return available_date

#TODO REFA, szczegolnie dla userow
def get_available_hours_in_day(self, user, date):
    visits = Visit.objects.filter(Q(user=self.user, client=self.client, start__year=date.year, start__month=date.month, start__day=date.day) |
                                  Q(user=self.user, client=self.client, end__year=date.year, end__month=date.month, end__day=date.day))

    start_work = timezone.make_aware(datetime.combine(date,self.work_time.start_time), timezone.get_default_timezone())
    end_work = timezone.make_aware(datetime.combine(date,self.work_time.end_time), timezone.get_default_timezone())

    avaliable_intervals = interval.closedopen(start_work, end_work)

    for visit in visits:
        visit = interval.closedopen(visit.start, visit.end)
        avaliable_intervals -= visit

    return_dict = {}
    list_of_hours = list_of_times_generate(start_work, end_work)

    for hour in list_of_hours:
        key = hour.strftime("%H:%M")
        if hour in avaliable_intervals:
            return_dict[key] = True
        else:
            return_dict[key] = False

    return return_dict

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

def list_of_times_generate(start, end=False, step=15):
    """ Generate list of datetime time objects from start to end with default step = 15 min. """

    list = []
    flag_time = start
    while flag_time <= end:
        list.append(flag_time)
        flag_time += timedelta(minutes=step)
    return list

def time_options(hours=8, sec=False):
    """ Generating list of times from 0 to hours (max:24). Step = 15min"""
    start = datetime(1, 1, 1, 0, 0, 0)
    end = start + timedelta(hours=hours)
    list = list_of_times_generate(start, end)

    choices = []
    for hour in list:
        if hour != datetime(1, 1, 2, 0, 0):
            label = hour.strftime("%H:%M")
        else:
            label = "24:00"
            hour = datetime(1, 1, 1, 23, 59, 59)

        if sec == True:
            value = hour.time()
        else:
            value = hour.strftime("%H:%M")

        choices.append([value, label])

    return choices




class Schedule:

    def __init__(self, username):
        self.title = None
        self.navigation = None
        self.username = username
        self.user = get_object_or_404(User, username__iexact=username)
        self.work_time = get_object_or_404(WorkTime, user=self.user)


    def display(self, year, week):
        self.title = "Title"
        self.days = self.get_dates_from_week(year, week)
        prev_date = self.days[0] - timedelta(days=7)
        next_date = self.days[0] + timedelta(days=7)
        self.navigation = self.generate_navigation(prev_date, next_date)

        html_code = ''
        html_code += self.title_header()
        html_code += self.main_header()
        html_code += self.schedule_content()

        return(html_code)




    #niezmienne
    def title_header(self):
        """ Method generates header of shedule with title and navigation """

        html_code = f'<div class="header"><ul>'
        if self.navigation:
            html_code += f'<a href="{self.navigation["prev_url"]}"><li class="prev">&#10094;</li></a>' \
                         f'<a href="{self.navigation["next_url"]}"><li class="next">&#10095;</li></a>'
        if self.title:
            html_code += f'<li>{self.title}</li>'
        html_code += f'</ul></div>'

        return html_code


    def main_header(self):
        """ Method generates header of schedule with days, months and years for current week """

        html_code = '<div class="dates-header">'
        html_code += '<ul><li class="empty"></li>'

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

        html_code += f'<ul><li class="empty"></li>'
        for n, day in enumerate(self.days):
            css_class = []
            if is_holiday(day) or n == 6: css_class.append('red')
            if datetime.today().date() == day.date(): css_class.append('today')
            if n == years_and_months_to_display_dict[0]['count']: css_class.append('border-date')
            css_class = ' '.join(css_class)
            if css_class: css_class = f' class="{css_class}"'

            html_code += f'<li{css_class}><div>{day.day}' \
                         f'<br />{DAYS_OF_WEEK_SHORT[n]}</div></li>'
        html_code += f'</ul></div>'

        return html_code


    def generate_navigation(self, prev_date, next_date):
        """ Method gets dates and return urls for shedule navigation """
        navigation = {}
        navigation['prev_url'] = self.get_navigation_url(prev_date)
        navigation['next_url'] = self.get_navigation_url(next_date)

        return navigation


    def get_dates_from_week(self, year, week):
        """ Method gets year and week, and return 7 dates - days of this week """

        days = []
        for i in range(1, 8):
            days.append(datetime.fromisocalendar(year, week, i))
        return days


    def get_years_and_months(self, dates):
        """ Method generates list of months and years of current week. List is generated with dictionaries:
        {month, year, apperances}. Every week exist in one or two months. Dict gives information, how many days in week
        exist in this month, and how many in other. """

        dicts = []

        for date_ in dates:
            month = date_.month
            year = date_.year
            exist_in_dict = False

            for dict in dicts:
                if month == dict['month']:
                    dict['count'] += 1
                    exist_in_dict = True
                    break

            if not exist_in_dict:
                dicts.append({'month': date_.month, 'year': date_.year, 'count': 1})

        return dicts


    def get_navigation_url(self, date):
        # Place to create url with date
        return reverse('dashboard')















    def schedule_content(self):

        # Prepare variables for generate time column
        start = (datetime.combine(date(1,1,1), self.work_time.start_time))
        if self.work_time.end_time != time(23,59,00): end = (datetime.combine(date(1,1,1), self.work_time.end_time))
        else: end = datetime(1,1,2,0,0,0)
        time_list = list_of_times_generate(start, end)

        # Html_code
        html_code = f'<ul class="schedule-content"><li>'
        html_code += f'<ul class="hours">'
        html_code += self.generate_time_column(time_list)
        html_code += f'</ul></li>'

        for d, day in enumerate(self.days):
            html_code += f'<li><ul class=day-{d}>'
            html_code += self.generate_day_column(day, time_list)
            html_code += f'</ul></li>'

        html_code += f'</ul>'
        return html_code

    def generate_time_column(self, time_list):
        """ Generate html for time column """
        html_code = ''
        for hour in time_list:
            if hour != datetime(1,1,2,0,0,0):
                hour_display = hour.strftime("%H:%M")
            else:
                hour_display = "24:00"

            html_code += f'<li><span>{hour_display}</span></li>'
        return html_code

    def generate_day_column(self, day, time_list):
        """ Generate html for day column """
        html_code = ''
        if day.date() not in self.available_dates:
            for hour in time_list:
                html_code += f'<li>&nbsp;</li>'
        else:
            pass
        return html_code




    def display_week(self):
        html_code =''
        simple_duration = int(self.service.duration / timedelta(minutes=15))



        av_time = self.get_available_hours_in_day(day.date())
        for i, hour in enumerate(work_hours):
            if not av_time[hour.strftime("%H:%M")]:
                html_code += f'<li>&nbsp;</li>'
            else:
                id = d*100 + i
                full_term = datetime.combine(day, hour)
                flag = True
                for n in range(simple_duration):
                    try:
                        if av_time[work_hours[i+n].strftime("%H:%M")]:
                            continue
                    except: pass
                    flag = False
                if flag:
                    html_code += f'<a href="{self.get_term_url(full_term)}" onmouseover="on_hover({id}, {simple_duration})" onmouseout="out_hover({id}, {simple_duration})"><li class="avaliable" id="{id}">&nbsp;</li></a>'
                else:
                    html_code += f'<li class="avaliable" id="{id}">&nbsp;</li>'
            #html_code += f'<a href="{self.get_term_url(full_term)}"><li>{hour.strftime("%H:%M")}</li></a>'




        return html_code







    def get_term_url(self, term):
        """ Method return link for choose visit by clients """
        return reverse('client_app_confirm_visit', args=[self.username, self.service.id,
                                                         term.year, term.month, term.day, term.hour, term.minute])







class ClientAddVisitSchedule(Schedule):

    def display(self, year, week, service):
        self.title = service.name
        self.service = service
        self.days = self.get_dates_from_week(year, week)
        prev_date = self.days[0] - timedelta(days=7)
        next_date = self.days[0] + timedelta(days=7)
        self.navigation = self.generate_navigation(prev_date, next_date)
        self.available_dates = get_available_days_for_clients(self.username)


        html_code = ''
        html_code += self.title_header()
        html_code += self.main_header()
        html_code += self.schedule_content()

        return(html_code)

    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        return reverse('client_app_new_visit', args=[self.username, self.service.id, year, week])

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
        self.available_dates = get_available_days_for_clients(self.username)

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
        self.available_dates = get_available_days_for_clients(self.username)

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
    def display(self, year, week):
        self.title = "Terminarz"
        self.days = self.get_dates_from_week(year, week)
        prev_date = self.days[0] - timedelta(days=7)
        next_date = self.days[0] + timedelta(days=7)
        self.navigation = self.generate_navigation(prev_date, next_date)
        self.available_dates = get_available_days_for_clients(self.username)

        html_code = ''
        html_code += self.title_header()
        html_code += self.main_header()
        html_code += self.schedule_content()
        html_code += f'<a href="{reverse("dashboard_schedule", args=[datetime.today().year, datetime.today().month, datetime.today().day])}">aa</a>'
        return (html_code)

    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        return reverse('dashboard_schedule', args=[year, week])


class UserTwoDaysSchedule(Schedule):

    def display(self):
        self.title = "Terminarz"
        self.available_dates = get_available_days_for_clients(self.username)

        html_code = ''
        html_code += self.title_header()
        #html_code += self.main_header()
        #html_code += self.schedule_content()
        return (html_code)


class UserOneDaySchedule(Schedule):
    def display(self, year, month, day):
        self.title = "Dzien" #Data, nazwa dnia, czy dziś, czy święto
        prev_date = datetime(year, month, day) - timedelta(days=1)
        next_date = datetime(year, month, day) + timedelta(days=1)

        self.navigation = self.generate_navigation(prev_date, next_date)

        html_code = ''
        html_code += self.title_header()
        #html_code += self.main_header()
        #html_code += self.schedule_content()
        return (html_code)

    def get_navigation_url(self, date):
        return reverse('dashboard_schedule', args=[date.year, date.month, date.day])