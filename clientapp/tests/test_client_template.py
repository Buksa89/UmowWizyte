from django.test import TestCase
from unittest import skip
from userapp.tests.base import BaseTest


class LoginTests(BaseTest):

    def test_login_template_display(self):
        """Czy templatka na stronie logowania prawidłowo się wczytuje?"""

        user = self.create_user()
        response = self.client.get(f'/{user}/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_login.html')

    def test_login_template_POST(self):
        """Czy formularz logownia wysyła POST na stronę logowania?"""
        user = self.create_user('active_1')
        response = self.client.post(f'/{user}/', data={})

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_login.html')

    def test_login_template_not_active(self):
        """Jeśli użytkownik nie jest aktywny, to jego terminarz jest zablokowany?"""
        user = self.create_user('not_active')
        response = self.client.get(f'/{user}/')

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_login_not_active.html')

class DashboardTests(BaseTest):
    def test_dashboard_template_display(self):
        """Url dahsboard wczytuje prawidłową templatkę"""
        user = self.create_user()
        client = self.create_client(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/panel/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_dashboard.html')

    def test_dashboard_template_POST(self):
        """Czy formularz dodawania wizyty wysyła post do pulpitu?"""
        user = self.create_user('active_1')
        response = self.client.post(f'/{user}/', data={})

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_login.html')


class AddVisitTests(BaseTest):

    def test_add_visit_url_template_display(self):
        """Czy url dodawania wizyty wyświetla prawidłową templatkę?"""
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        self.authorize_client(client)
        response = self.client.get(f'/{user}/nowa_wizyta/{service.id}/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_new_visit.html')


    def test_add_visit_extended_url_template_display(self):
        """Czy url rozszerzony url dodawania wizyty wyświetla prawidłową templatkę?"""
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        date = self.weeks['no_holiday']
        self.authorize_client(client)
        response = self.client.get(f"/{user}/nowa_wizyta/{service.id}/{date['year']}/{date['week']}/")
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_new_visit.html')

    def test_confirmation_visit_url_template_display(self):
        """Czy url potwierdzania wizyty wyświetla prawidłową templatkę?"""
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        date_time = self.date_time['regular']
        self.authorize_client(client)
        response = self.client.get(f"/{user}/nowa_wizyta/{service.id}/{date_time.year}/{date_time.month}/{date_time.day}/{date_time.hour}/{date_time.minute}/")
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_confirm_visit.html')

    @skip #TODO: redirect do dashboard
    def test_confirmation_visit_url_template_POST_display(self):
        """Czy url potwierdzania wizyty wysyła POST do siebie?"""
        user = self.create_user()
        client = self.create_client(user)
        service = self.create_service(user)
        date_time = self.date_time['regular']
        self.authorize_client(client)
        response = self.client.post(f"/{user}/nowa_wizyta/{service.id}/{date_time.year}/{date_time.month}/{date_time.day}/{date_time.hour}/{date_time.minute}/", data={})

        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_confirm_visit.html')