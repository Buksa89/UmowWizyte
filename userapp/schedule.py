from datetime import date, datetime, time, timedelta
from django.urls import reverse
from django.utils import timezone
import holidays
from django.contrib.auth.models import User
from django.db.models import Q
import portion as interval
from copy import deepcopy

from userapp.variables import DAYS_OF_WEEK_SHORT, MONTHS
from userapp.models import Client, Service, TimeOff, UserSettings, Visit, WorkTime

def not_naive(dat):
    return dat

class Schedule:

    def __init__(self, user, year, week):
        self.title= 'Terminarz'
        self.user = user
        self.days = self.get_dates_from_week(year, week)
        # Possible arguments:
            # full - from 00:00 to 24:00
            # work_time - only worktime in days range
            # visits - worktime + optional visits in days range
        self.time_range_type = 'visits'
        self.visible_visits = True
        # Possible arguments:
            # False - not dispaly available time
            # all - all fields avaliable
            # no_visits - avaliable all fields, expect visits
            # available - avaliable fields, expect time_off and visits
        self.available_time = False
        self.event_duration = timedelta(seconds=0)

        self.prepare_data()

    def is_holiday(self, date_):
        """ Function get date and return True if it is holiday """
        if date_ in holidays.Poland(): return True

    def get_dates_from_week(self, year, week):
        """ Method gets year and week, and return 8 dates - days of this week and day next. Next day is neccesary to
         check if long visit can be booked at last day"""

        days = []
        for i in range(1, 8):
            days.append(date.fromisocalendar(year, week, i))
        next_day = days[-1] + timedelta(days=1)
        days.append(next_day)
        return days

    def generate_navigation(self):
        """ Method gets dates and return urls for shedule navigation """
        navigation = {}
        dates = {}
        dates['prev'], dates['next'] = self.get_navigation_dates()

        for key, date in dates.items():
            navigation[key] = self.get_navigation_url(date)
        return navigation

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

    def get_time_steps(self, type_, off_intervals):
        """ Method return list of quarters of hours to display in schedule. """
        start_time = None
        end_time = None
        times = []

        if type_ == 'full':
            times.append(timedelta(seconds=0))
            times.append(timedelta(hours=24))

        else:
            for day in self.days[:-1]:
                datetime_day = not_naive(datetime.combine(day, time.min))
                day_interval = interval.closedopen(datetime_day, datetime_day+timedelta(days=1))
                on_intervals = day_interval - off_intervals
                if on_intervals != interval.empty():
                    for on_interval in on_intervals:
                        times.append(on_interval.lower - not_naive(datetime.combine(day, time.min)))
                        times.append(on_interval.upper - not_naive(datetime.combine(day, time.min)))

                if type_ == 'visits':
                    for visit in self.visits:
                        if visit.start in day_interval:
                            times.append(visit.start - datetime_day)
                        if visit.end in day_interval:
                            times.append(visit.end - datetime_day)

        if times:
            start_time = min(times)
            end_time = max(times)
        else:
            start_time = timedelta(hours=8)
            end_time = timedelta(hours=16)

        time_steps = self.generate_time_steps(start_time, end_time)
        return time_steps, start_time, end_time

    def generate_time_steps(self, min, max):
        time_step = timedelta(minutes=15)
        list_ = []
        flag_time = min
        while flag_time <= max:
            list_.append((datetime.min + flag_time).time())
            flag_time += time_step
        return list_

    def get_current_time(self):
        nn_now = not_naive(datetime.now())
        rounded_now = datetime(nn_now.year, nn_now.month, nn_now.day, nn_now.hour, nn_now.minute - nn_now.minute % 15)
        rounded_now = not_naive(rounded_now)
        return rounded_now

    def day_intervals(self):
        day_visits_intervals = []
        day_off_intervals = []
        day_past_interval = []
        day_visits = []
        day_intervals =[]

        for n, day in enumerate(self.days):
            datetime_day = not_naive(datetime.combine(day, time.min))
            day_interval = interval.closedopen(datetime_day, datetime_day+timedelta(days=1))
            day_intervals.append(day_interval)
            visits_intervals = day_interval & self.visits_intervals
            day_visits_intervals.append((visits_intervals))
            off_intervals = day_interval & self.off_intervals
            day_off_intervals.append((off_intervals))
            past_interval = day_interval & self.past_interval
            day_past_interval.append((past_interval))

            day_visits.append([])

            for visit in self.visits:
                visit_interval = interval.closedopen(visit.start, visit.end) & day_interval
                if visit_interval != interval.empty():
                    day_visit = deepcopy(visit)
                    day_visit.start = visit_interval.lower
                    day_visit.end = visit_interval.upper

                    day_visits[n].append(day_visit)

        """for visit in day_visits:
            if visit:
                print (visit[0].start)"""

        return day_intervals, day_visits_intervals, day_off_intervals, day_past_interval, day_visits


    # To change in subclasses
    def get_navigation_dates(self):
        prev = self.days[0] - timedelta(days=7)
        next = self.days[0] + timedelta(days=7)
        return prev, next

    def get_navigation_url(self, date):
        year = date.year
        week = date.isocalendar()[1]
        return reverse('schedule_date', args=[year, week])

    # Data from DB
    def get_work_time(self):
        work_time_result = []
        intervals = interval.empty()
        reverse_intervals = interval.closedopen(not_naive(datetime.combine(self.days[0], time.min)),
                                                not_naive(datetime.combine(self.days[-1], time.max)))

        holidays_work = UserSettings.objects.get(user=self.user).holidays

        for day in self.days:

            if not self.is_holiday(day) or holidays_work:
                day_number = day.weekday()
                work_time = WorkTime.objects.filter(user=self.user, day_of_week=day_number)
                if work_time:
                    for work_hours in work_time:
                        day_datetime = datetime.combine(day, time.min)
                        work_hours.start = not_naive(day_datetime + work_hours.start)
                        work_hours.end = not_naive(day_datetime + work_hours.end)
                        work_time_result.append(work_hours)

                        intervals |= interval.closedopen(work_hours.start, work_hours.end)


        reverse_intervals -= intervals

        return work_time_result, reverse_intervals

    def get_time_off(self):
        """ Method get time off for days from database"""

        first_day = not_naive(datetime.combine(self.days[0], time.min))
        last_day = not_naive(datetime.combine(self.days[-1], time.max))
        intervals = interval.empty()

        times_off = TimeOff.objects.filter(
                Q(user=self.user, start__range=[first_day, last_day]) |
                Q(user=self.user, end__range=[first_day, last_day]))

        for time_off in times_off:
            intervals |= interval.closedopen(time_off.start, time_off.end)

        return times_off, intervals

    def get_visits(self):
        """ Method get visits for days from database"""
        first_day = datetime.combine(self.days[0], time.min)
        last_day = datetime.combine(self.days[-1], time.max)
        intervals = interval.empty()

        visits = Visit.objects.filter(~Q(is_confirmed=True, is_available=False) &
                                      (Q(user=self.user, start__range=[first_day, last_day]) |
                                       Q(user=self.user, end__range=[first_day, last_day])))


        for visit in visits:
            intervals |= interval.closedopen(visit.start, visit.end)


        return visits, intervals

    def get_off_intervals(self):
        self.work_time, work_time_reverse_intervals = self.get_work_time()
        self.time_off, time_off_intervals = self.get_time_off()

        total_off_intervals = work_time_reverse_intervals | time_off_intervals

        return total_off_intervals

    def get_days_interval(self):
        start = datetime.combine(self.days[0], time.min)
        end = datetime.combine(self.days[-1], time.max)

        return interval.closedopen(start, end)

    def prepare_data(self):
        self.visits, self.visits_intervals = self.get_visits()
        self.past_interval = interval.openclosed(-interval.inf, datetime.now())
        self.off_intervals = self.get_off_intervals()
        self.days_interval = self.get_days_interval()
        self.day_intervals, self.day_visits_intervals, self.day_off_intervals, self.day_past_interval, self.day_visits = self.day_intervals()
        self.time_steps, self.start_day, self.end_day = self.get_time_steps(self.time_range_type, self.off_intervals)
        self.navigation = self.generate_navigation()


    #HTML
    def display(self):

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

    def date_header(self):
        """ Method generates header of schedule with date """
        # get numbers of month appearance in days of current week
        html_code = ''
        years_and_months_dict = self.get_years_and_months(self.days[:-1])

        html_code += f'<span class="month-slot" aria-hidden="true" style="grid-column: times; grid-row: months;"></span>'
        end = 0
        for n, data in enumerate(years_and_months_dict):
            start = end + 1
            html_code += f'<span class="month-slot" aria-hidden="true" style="grid-column: day-{start}-start / '
            end = start + data['count'] - 1
            html_code += f'day-{end}-end; grid-row: months;">{data["year"]}<br/>{MONTHS[data["month"]-1]}</span>'

        html_code += f'<span class="day-number-slot" aria-hidden="true" style="grid-column: times; grid-row: day-number;"></span>'
        for n, day in enumerate(self.days[:-1]):
            html_code += f'<span class="day-number-slot" aria-hidden="true" style="grid-column: day-{n+1}; grid-row: day-number;">{day.day}</span>'

        return html_code

    def days_header(self):
        """ Method generates header of schedule with days """
        html_code = ''

        html_code += f'<span class="day-slot" aria-hidden="true" style="grid-column: times; grid-row: days;"></span>'
        html_code += f'<span class="day-slot menu-line-strong" aria-hidden="true" style="grid-column: times / day-7; grid-row: days;"></span>'
        for n, day in enumerate(self.days[:-1]):
            span_classes = ''
            span_content = DAYS_OF_WEEK_SHORT[n]
            if self.is_holiday(day) or day.weekday()==6:
                span_classes +=(' holiday')
            if day == date.today():
                span_classes +=(' today')
                span_content = f'<span>{span_content}</span>'

            html_code += f'<span class="day-slot{span_classes}" aria-hidden="true" style="grid-column: day-{n+1}; grid-row: days;">{span_content}</span>'

        return html_code

    def get_time_column_html(self):
        """ Generate html for time column """
        html_code = ''
        for n, hour in enumerate(self.time_steps):
            line_class = 'time-line'
            time_class = ''
            hour_css = hour.strftime('%H%M')
            if hour.minute == 15 or hour.minute==45:
                hour_display = ''
            else:
                hour_display = hour.strftime('%H:%M')
                if hour.minute == 00:
                    time_class = ' strong'

            if hour == time.min and n+1 == len(self.time_steps):
                hour_css = "2400"
                hour_display = ''

            html_code += f'<div class="{line_class}" style="grid-column: times / day-7; grid-row: time-{hour_css}"></div>'
            html_code += f'<h2 class="time-slot{time_class}" style="grid-row: time-{hour_css};"><span>{hour_display}</span></h2>'



        return html_code

    def schedule_content(self):
        html_code = ''

        for d, day in enumerate(self.days[:-1]):
            total_off = (self.day_off_intervals[d] | self.day_past_interval[d]) - self.day_visits_intervals[d]
            for time_off in total_off:
                html_code += self.generate_time_off_content(d, time_off)

            for visit in self.day_visits[d]:
                html_code += self.generate_visit_content(d, visit)


            current_time = self.get_current_time()
            if current_time.date() == day:
                str_time = current_time.strftime('%H%M')
                html_code += f'<div class="line-off day-{d + 1}" style="grid-column: day-{d + 1}; grid-row: time-{str_time};">' \
                             f'</div>'

        if self.available_time:
            if self.available_time == 'available':
                available_intervals = self.days_interval - self.off_intervals - self.past_interval - self.visits_intervals

            elif self.available_time == 'no_visits':
                available_intervals = self.days_interval - self.visits_intervals

            else: available_intervals = self.days_interval

            html_code += self.generate_available_time_content(available_intervals)

        return html_code


    def generate_time_off_content(self, d, times_off):
        html_code = ''
        if not times_off.empty:
            start_row = times_off.lower.strftime("%H%M")
            end_row = times_off.upper.strftime("%H%M")
            if end_row == "0000": end_row = "2400"

            html_code += f'<div class="time-off day-{d + 1}" style="grid-column: day-{d + 1}; grid-row: time-{start_row} / time-{end_row};">' \
                         f'</div>'

        return html_code

    #TODO: Niepotwierdzone niech maja link do potwierdzenia
    def generate_visit_content(self, d, visit):
        start_row = visit.start.strftime("%H%M")
        end_row = visit.end.strftime("%H%M")
        if end_row == '0000': end_row = '2400'
        color_light = ''
        if not visit.is_confirmed:
            color_light = ' color-light'
        client_url = reverse('clients_data', args=[visit.client.id])
        html_code = f'<div class="session day-{d+1}{color_light}" style="grid-column: day-{d+1}; grid-row: time-{start_row} / time-{end_row};">' \
                     f'<h3 class="session-title"><a href="{visit.get_display_url()}">{visit.name}</a></h3>' \
                     f'<p><a href="{client_url}">{visit.client.name}<br />' \
                     f'{visit.client.surname}<br /></a>' \
                     f'<a href="tel:{visit.client.phone_number}">{visit.client.phone_number}</a></p>' \
                     f'<p>{visit.description}</p>' \
                     f'</div>'

        return html_code

    def generate_available_time_content(self, available_time):

        html_code = ''

        rows_number = (self.end_day - self.start_day) / timedelta(minutes=15)
        quarters_event_duration = self.event_duration / timedelta(minutes=15)

        time_ = self.days_interval.lower
        start_time = time_
        end_time = datetime.combine(self.days[-2], time.max)
        field_id = 1
        last_id = None
        last_time = None
        url = None
        last_correct_time = None
        while time_ < end_time:
            if time_ in available_time:
                event_interval = interval.closedopen(time_, time_+self.event_duration)
                row = time_.strftime("%H%M")
                column = (time_ - start_time).days + 1
                
                if event_interval in available_time:
                    last_correct_time = time_
                    url = self.get_available_url(last_correct_time, self.days[column - 1])
                    html_code += f'<a id="{field_id}" href="{url}" ' \
                                 f'class="available day-{column}" style="grid-column: day-{column}; ' \
                                 f'grid-row: time-{row}" ' \
                                 f'onmouseover="on_hover({field_id}, {quarters_event_duration})" ' \
                                 f'onmouseout="out_hover({field_id}, {quarters_event_duration})">' \
                                 f'</a>'

                    last_id = field_id
                    last_correct_time = time_
                    last_time = time_

                elif last_id and last_time == time_ - timedelta(minutes=15):
                    html_code += f'<a id="{field_id}" href="{url}" ' \
                                 f'class="available day-{column}" style="grid-column: day-{column}; ' \
                                 f'grid-row: time-{row}" ' \
                                 f'onmouseover="on_hover({last_id}, {quarters_event_duration})" ' \
                                 f'onmouseout="out_hover({last_id}, {quarters_event_duration})">' \
                                 f'</a>'
                    last_time = time_

                else:
                    html_code += f'<div id="{field_id}" ' \
                                 f'class="day-{column}" style="grid-column: day-{column}; ' \
                                 f'grid-row: time-{row}">' \
                                 f'</div>'

            field_id += 1
            time_ += timedelta(minutes=15)

        return html_code