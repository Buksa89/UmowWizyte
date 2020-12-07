from datetime import date, datetime, time, timedelta
from django.contrib.auth.models import User
from django.db.models import Q
from django.shortcuts import get_object_or_404
from django.urls import reverse
from django.utils import timezone

import holidays
import portion as interval

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

def is_holiday(date):
    """ Function get date and return True if it is holiday """
    if date in holidays.Poland(): return True

def is_free_day(date, work_time):
    """ Function get date and return True if it is not working day """
    if not work_time.__dict__[date.strftime("%A").lower()]: return True




class Schedule:

    def __init__(self, work_time, username, available_dates, client):
        self.title = None
        self.navigation = None
        self.username = username

        """self.client = client
        self.start_work_time = work_time.start_time
        self.end_work_time = work_time.end_time
        self.work_time = work_time
        self.user = get_object_or_404(User, username__iexact=username)
        self.service = get_object_or_404(Service, id=service_id, user=self.user)
        self.username = username
        self.available_dates = available_dates
        self.year = year
        self.week = week
        self.day = """

    def display(self, year, week, service, work_time, available_dates, client):
        self.title = service.name
        self.service = service
        days = self.get_dates_from_week(year, week)
        prev_date = days[0] - timedelta(days=7)
        next_date = days[0] + timedelta(days=7)
        self.navigation = self.generate_navigation(prev_date, next_date)


        html_code = ''
        html_code += self.title_header()
        html_code += self.week_header(days)

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


    def week_header(self, days):
        """ Method generates header of schedule with days, months and years for current week """

        html_code = '<div class="dates-header">'
        html_code += '<ul><li class="empty"></li>'

        # Year and month display
        # get numbers of month appearance in days of current week
        years_and_months_to_display_dict = self.get_years_and_months(days)

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
        for n, day in enumerate(days):
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


    #ZMIENNE:
    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        return reverse('client_app_new_visit', args=[self.username, self.service.id, year, week])











    def display_week(self):
        html_code = self.display_header()
        html_code += self.display_dates_header()
        html_code += f'<ul class="schedule-content"><li><ul class="hours">'
        # Hours column
        hour = self.start_work_time
        work_hours = []

        while hour <= self.end_work_time:
            work_hours.append(hour)
            html_code += f'<li><span>{hour.strftime("%H:%M")}</span></li>'

            hour = (datetime.combine(date.today(), hour) + timedelta(minutes=15)).time()

            #TODO problemy z godziną 24:00
            if hour == time(0,0,0) : break;

        html_code += '</ul></li>'
        simple_duration = int(self.service.duration / timedelta(minutes=15))

        for d, day in enumerate(self.day):
            if day.date() not in self.available_dates:
                html_code += f'<li><ul class=day-{d}>'
                for hour in work_hours:
                    html_code += f'<li>&nbsp;</li>'
                html_code += '</ul></li>'
            else:
                html_code += f'<li><ul class=day-{d}>'

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
                html_code += '</ul></li>'

        html_code += f'</ul>'



        return html_code







    def get_term_url(self, term):
        """ Method return link for choose visit by clients """
        return reverse('client_app_confirm_visit', args=[self.username, self.service.id,
                                                         term.year, term.month, term.day, term.hour, term.minute])

    def get_available_hours_in_day(self, date):
        visits = Visit.objects.filter(Q(user=self.user, client=self.client, start__year=date.year, start__month=date.month, start__day=date.day) |
                                      Q(user=self.user, client=self.client, end__year=date.year, end__month=date.month, end__day=date.day))

        start_work = timezone.make_aware(datetime.combine(date,self.work_time.start_time), timezone.get_default_timezone())
        end_work = timezone.make_aware(datetime.combine(date,self.work_time.end_time), timezone.get_default_timezone())

        avaliable_intervals = interval.closedopen(start_work, end_work)

        for visit in visits:
            visit = interval.closedopen(visit.start, visit.end)
            avaliable_intervals -= visit

        return_dict = {}
        list_of_hours = generate_hours_list(start_work, end_work, 15)

        for hour in list_of_hours:
            key = hour.strftime("%H:%M")
            if hour in avaliable_intervals:
                return_dict[key] = True
            else:
                return_dict[key] = False

        return return_dict

def generate_hours_list(start, end, step=15):
    list = []
    flag_time = start
    while flag_time <= end:
        list.append(flag_time)
        flag_time += timedelta(minutes=step)

    return list



class ClientSchedule(Schedule):
    pass