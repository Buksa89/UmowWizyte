from datetime import date, datetime, timedelta
from django.test import TestCase
from unittest import skip
from ..forms import ClientLoginForm
from userapp.models import WorkTime
from userapp.tests.base import BaseTest


class DashboardTests(BaseTest):

    def test_form_display(self):
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/panel/')

        self.assertContains(response, 'choose-visit-form')

    def test_dashboard_POST_redirect(self):
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        service = self.create_service(user)
        response = self.client.post(f'/{user}/panel/', data={'service':service.id})

        self.assertRedirects(response, f'/{user}/nowa_wizyta/{service.id}/')

    def test_correct_services_in_form(self):
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        services = []
        services_types = ['short_1', 'short_2', 'long_1', 'long_2']
        for service in services_types:
            services.append(self.create_service(user, service))
        response = self.client.get(f'/{user}/panel/')

        for service in services:
           self.assertContains(response, f'<option value="{service.id}">{service.name}</option>')


    def test_only_active_services_in_form(self):
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        service_active = self.create_service(user, 'short_1')
        service_not_active = self.create_service(user, 'not_active')
        response = self.client.get(f'/{user}/panel/')

        self.assertContains(response, f'<option value="{service_active.id}">{service_active.name}</option>')
        self.assertNotContains(response, f'<option value="{service_not_active.id}">{service_not_active.name}</option>')


    def test_incorrect_services_in_form(self):
        user1 = self.create_user('active_1')
        user2 = self.create_user('active_2')
        service_user_2 = self.create_service(user2, 'short_1')
        client = self.create_client(user1)
        self.authorize_client(client)
        response = self.client.get(f'/{user1}/panel/')

        self.assertNotContains(response, f'<option value="{service_user_2.id}">{service_user_2.name}</option>')


    def test_cant_get_other_users_service(self):
        user1 = self.create_user('active_1')
        user2 = self.create_user('active_2')
        service_user_2 = self.create_service(user2)
        client = self.create_client(user1)
        self.authorize_client(client)
        response = self.client.get(f'/{user1}/nowa_wizyta/{service_user_2.id}/')

        self.assertEqual(response.status_code, 404)


    def test_cant_get_not_active_service(self):
        user = self.create_user()
        service = self.create_service(user,'not_active')
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/nowa_wizyta/{service.id}/')

        self.assertEqual(response.status_code, 404)

    def test_empty_visits_lists(self):
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/panel/')

        self.assertContains(response, 'Nie jesteś umówiony na żadną wizytę')

    def test_visits_lists(self):
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/panel/')

        self.assertContains(response, 'Nie jesteś umówiony na żadną wizytę')


class AddVisitTest(BaseTest):
    def test_schedule_navigation_header(self):
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        date = self.weeks['no_holiday']
        self.authorize_client(client)
        response = self.client.get(f"/{user}/nowa_wizyta/{service.id}/{date['year']}/{date['week']}/")

        self.assertContains(response, f'<li>{service.name}</li>')
        self.assertContains(response, f'<a href="/{user}/nowa_wizyta/{service.id}/{date["year"]}/{date["week"]-1}/"><li class="prev">&#10094;</li></a>')
        self.assertContains(response, f'<a href="/{user}/nowa_wizyta/{service.id}/{date["year"]}/{date["week"]+1}/"><li class="next">&#10095;</li><')

    def test_days_header(self):
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        date = self.weeks['no_holiday']
        self.authorize_client(client)
        response = self.client.get(f"/{user}/nowa_wizyta/{service.id}/{date['year']}/{date['week']}/")
        days = list(range(24,31))
        day_names = ['Pn','Wt','Śr','Cz','Pt','So','Nd']

        for day, name in zip(days, day_names):
            self.assertContains(response, f'{day}<br />{name}')

    def test_months_header(self):
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        date = self.weeks['with_holiday']
        self.authorize_client(client)
        response = self.client.get(f"/{user}/nowa_wizyta/{service.id}/{date['year']}/{date['week']}/")

        self.assertContains(response, f'<li style="width:12.5%" class="">2022<br />Październik</li>'
                                      f'<li style="width:75.0%" class="border-date">2022<br />Listopad</li></ul><ul>')

    def test_hours_display(self):
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        self.authorize_client(client)
        response = self.client.get(f"/{user}/nowa_wizyta/{service.id}/")
        worktime = WorkTime.objects.get(user=user)

        hour = worktime.start_time
        work_hours = []
        while hour <= worktime.end_time:
            work_hours.append(hour.strftime("%H:%M"))
            hour = (datetime.combine(date.today(), hour) + timedelta(minutes=15)).time()
        for hour in work_hours:
            self.assertContains(response, f'<span>{hour}</span>')


    def test_highlight_today(self):
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        self.authorize_client(client)
        response = self.client.get(f"/{user}/nowa_wizyta/{service.id}/")

        self.assertContains(response, f'today')


    def test_highlight_sundays_and_holidays(self):
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        date = self.weeks['with_holiday']
        self.authorize_client(client)
        response = self.client.get(f"/{user}/nowa_wizyta/{service.id}/{date['year']}/{date['week']}/")

        self.assertContains(response, f'<li class="red border-date"><div>1<br />Wt</div></li>')
        self.assertContains(response, f'<li class="red"><div>6<br />Nd</div></li>')


    def test_days_access(self):
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        self.authorize_client(client)
        date = self.weeks['with_holiday']
        worktime = WorkTime.objects.get(user=user)
        self.work_time_save(worktime,'weekend_holidays_free')
        response = self.client.get(f"/{user}/nowa_wizyta/{service.id}/{date['year']}/{date['week']}/")
        work_list = [True, False, True, True, True, False, False]
        for day, work in enumerate(work_list):
            if work:
                self.assertContains(response, f'day-{day}><a')
            else:
                self.assertContains(response, f'day-{day}><li>&nbsp;')


    def test_other_users_services(self):
        user_1 = self.create_user('active_1')
        user_2 = self.create_user('active_2')
        client = self.create_client(user_1)
        service = self.create_service(user_2)
        self.authorize_client(client)
        response = self.client.get(f"/{user_1}/nowa_wizyta/{service.id}/")

        self.assertEqual(response.status_code, 404)


    def test_correct_url_to_visit_confirm(self):
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        self.authorize_client(client)
        date = self.weeks['no_holiday']
        worktime = WorkTime.objects.get(user=user)
        self.work_time_save(worktime,'weekend_holidays_free')
        response = self.client.get(f"/{user}/nowa_wizyta/{service.id}/{date['year']}/{date['week']}/")
        work_list = [True, True, True, True, True, False, False]

        days = []
        days.append(datetime.fromisocalendar(date['year'], date['week'], 1))
        for i in range(0, 6):
            days.append(days[-1] + timedelta(days=1))

        for day_nr, work in enumerate(work_list):
            if work:
                day = days[day_nr].day
                month = days[day_nr].month
                year = days[day_nr].year
                hour = int(worktime.start_time.split(':')[0])
                minute = int(worktime.start_time.split(':')[1])
                self.assertContains(response, f'day-{day_nr}><a href="/{user}/nowa_wizyta/{service.id}/'
                                              f'{year}/{month}/{day}/{hour}/{minute}"')




    def test_booked_time_not_avaliable(self):
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        visit = self.create_visit(user, client, 'short_1')
        self.authorize_client(client)
        date = self.weeks['no_holiday']
        worktime = WorkTime.objects.get(user=user)
        self.work_time_save(worktime,'weekend_holidays_free')
        response = self.client.get(f"/{user}/nowa_wizyta/{service.id}/{date['year']}/{date['week']}/")

        day = visit.start.day
        month = visit.start.month
        year = visit.start.year
        not_avaliable_time = visit.start

        while True:
            if not_avaliable_time == visit.end: break;
            hour = int(not_avaliable_time.strftime('%H'))
            minute = int(not_avaliable_time.strftime('%M'))
            self.assertNotContains(response, f'<a href="/{user}/nowa_wizyta/{service.id}/{year}/{month}/{day}/{hour}/{minute}"')
            not_avaliable_time += timedelta(minutes=15)






class LoginTests(BaseTest):

    def test_incorrect_data_errors_display(self):

        user_active_1 = self.create_user('active_1')
        user_active_2 = self.create_user('active_2')

        client_of_user_1 = self.create_client(user_active_1, 'active_1')
        client_not_active = self.create_client(user_active_1, 'not_active')
        client_of_user_2 = self.create_client(user_active_2, 'active_2')

        data_results = [{'data': {'phone_number': client_of_user_1.phone_number, 'pin': self.ANY_PIN}, 'message': 'Dane nieprawidłowe'},
                        {'data': {'phone_number': self.ANY_PHONE, 'pin': client_of_user_1.pin}, 'message': 'Nie ma takiego numeru'},
                        {'data': {'phone_number': client_of_user_2.phone_number, 'pin': client_of_user_2.pin}, 'message': 'Nie ma takiego numeru'},
                        {'data': {'phone_number': client_not_active.phone_number, 'pin': client_not_active.pin}, 'message': 'Konto zablokowane'},
                        {'data': {'phone_number': self.EMPTY, 'pin': client_of_user_1.pin}, 'message': 'Podaj numer telefonu'},
                        {'data': {'phone_number': client_of_user_1.phone_number, 'pin': self.EMPTY}, 'message': 'Podaj pin'},
                        {'data': {'phone_number': client_of_user_1.phone_number, 'pin': self.WRONG_DATA}, 'message': 'Podaj prawidłowy pin'},
                        {'data': {'phone_number': self.WRONG_DATA, 'pin': client_of_user_1.pin}, 'message': 'Podaj prawidłowy numer telefonu'},]

        for data_result in data_results:
            response = self.client.post(f'/{user_active_1}/', data=data_result['data'])
            self.assertContains(response, data_result['message'])

    def test_incorrect_data_not_authorize(self):
        user = self.create_user()
        client = self.create_client(user)
        self.client.post(f'/{user}/', data={'phone_number': client.phone_number, 'pin': self.ANY_PIN})
        self.client.post(f'/{user}/', data={'phone_number': self.ANY_PHONE, 'pin': client.pin})

        self.assertNotIn('client_authorized', self.client.session)


    def test_not_active_client_not_auhorize(self):
        user = self.create_user()
        client = self.create_client(user, 'not_active')
        self.client.post(f'/{user}/', data={'phone_number': client.phone_number, 'pin': client.pin})

        self.assertNotIn('client_authorized', self.client.session)


    def test_correct_client_auhorized(self):
        user = self.create_user()
        client = self.create_client(user)
        self.client.post(f'/{user}/', data={'phone_number': client.phone_number, 'pin': client.pin})
        self.assertIn('client_authorized', self.client.session)
        correct_session = {'phone':client.phone_number,'user':user.username}
        self.assertEqual(self.client.session['client_authorized'], correct_session)\


    def test_remove_client_from_session_after_logout(self):
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/logout/')
        self.assertNotIn('client_authorized', self.client.session)


    """ Redirects tests """

    def test_redirect_to_login_url_when_client_not_authorized_for_this_user(self):
        user = self.create_user('active_1')
        user2 = self.create_user('active_2')
        client = self.create_client(user2)
        self.authorize_client(client)

        # TODO: Po dodaniu kazdej podstrony nalezy uzupelnic tą funkcję
        subpages = ['panel']

        for subpage in subpages:
            response = self.client.get(f'/{user}/{subpage}/')

            self.assertRedirects(response, f'/{user}/')


    def test_redirect_from_login_to_dashboard_url_when_client_authorized_for_this_user(self):
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/')

        self.assertRedirects(response, f'/{user}/panel/')


    def test_redirect_to_dashboard_after_login(self):
        user = self.create_user()
        client = self.create_client(user)
        response = self.client.post(f'/{user}/', data={'phone_number':client.phone_number, 'pin':client.pin})

        self.assertRedirects(response, f'/{user}/panel/')


    def test_redirect_after_logout(self):
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/logout/')

        self.assertRedirects(response, f'/{user}/')