from django.contrib.auth import authenticate
from django.contrib.auth.models import User
from django.test import TestCase
from unittest import skip
from ..forms import ClientChooseVisitForm
from ..models import Client
from .base import BaseTest

class ClientDashboardTemplateTests(BaseTest):
    def test_client_dashboard_template(self):
        self.authorize_client()
        response = self.client.get(f'/{self.user}/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_app/client_dashboard.html')

    def test_new_visit_template_post(self):
        self.authorize_client()
        response = self.client.post(f'/{self.user}/nowa_wizyta/', data={'service':'service'})
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'client_app/client_app_new_visit_step_1.html')

    def test_new_visit_template_no_post(self):
        self.authorize_client()
        response = self.client.get(f'/{self.user}/nowa_wizyta/')
        self.assertNotEqual(response.status_code, 200)



class ClientDashboardFormTests(BaseTest):
   def test_display_form(self):
        self.authorize_client()
        response = self.client.get(f'/{self.user}/')
        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['form'], ClientChooseVisitForm)




class ClientDashboardViewTests(BaseTest):
    @skip #TODO: sprawdź czy wyświetlają się prawidłowe wiziyty do wyboru w formularzu
    def test_correct_visits_in_form(self):
        self.authorize_client()
        response = self.client.get(f'/{self.user}/')
        #print(response.content.decode())

    # TODO: sprawdź czy wyświetlają się prawidłowe wiziyty na liście