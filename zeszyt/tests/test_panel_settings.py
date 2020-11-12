from datetime import timedelta
from django.contrib.auth.models import User
from django.core.exceptions import ValidationError
from django.test import TestCase
from unittest import skip
from ..forms import AddServiceForm
from ..models import Service
from .base import BaseTest

class PanelSettingsTemplateTests(BaseTest):

    def test_panel_settings_template(self):
        self.authorize_user()
        response = self.client.get('/ustawienia/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'panel/settings.html')

    def test_panel_settings_template_POST(self):
        self.authorize_user()
        response = self.client.post('/ustawienia/', data={})
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'panel/settings.html')


class ServicesModelTests(TestCase):

    def test_service_saving(self):
        user = User.objects.create()
        service = Service.objects.create(duration=timedelta(hours=1), name='usługa', user=user)
        self.assertEqual(service, Service.objects.first())

    def test_cannot_save_empty_user(self):
        with self.assertRaises(ValidationError):
            service = Service(duration=timedelta(hours=1), name='usługa')
            service.full_clean()

    def test_cannot_save_empty_name(self):
        user = User.objects.create()
        with self.assertRaises(ValidationError):
            service = Service(duration=timedelta(hours=1), user=user)
            service.full_clean()

    def test_cannot_save_empty_duration(self):
        user = User.objects.create()
        with self.assertRaises(AttributeError):
            Service.objects.create(name='usługa', user=user)


    def test_duplicate_service_name(self):
        user = User.objects.create()
        Service.objects.create(user=user, duration=timedelta(hours=1), name='usługa')
        with self.assertRaises(ValidationError):
            service = Service(user=user, duration=timedelta(hours=1), name='usługa')
            service.full_clean()

    def test_long_service_name(self):
        name = "Muvaffakiyetsizleştiricilestiriveremeyebileceklerimizdenmissinizcesine"
        user = User.objects.create()
        with self.assertRaises(ValidationError):
            service = Service(user=user, duration=timedelta(hours=1), name=name)
            service.full_clean()

    def test_cannot_save_wrong_duration(self):
        user = User.objects.create()
        with self.assertRaises(ValidationError):
            service = Service(user=user, duration='ffff', name='usługa')
            service.full_clean()

    def test_service_is_related_to_user(self):
        user = User.objects.create()
        service = Service(duration=timedelta(hours=1), name='usługa')
        service.user = user
        service.save()
        self.assertIn(service, user.service_set.all())

    def test_CAN_save_same_service_to_different_users(self):
        user1 = User.objects.create()
        user2 = User.objects.create(username='2')
        Service.objects.create(user=user1, duration=timedelta(hours=1), name='usługa')
        service = Service(user=user2, duration=timedelta(hours=1), name='usługa')
        service.full_clean()  # Nie powinien być zgłoszony


class PanelSettingsFormTests(BaseTest):
    def test_service_uses_item_form(self):
        self.authorize_user()
        response = self.client.get('/ustawienia/')
        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.context['service_form'], AddServiceForm)

    def test_add_service_form_validation_for_blank_fields(self):
        form = AddServiceForm(data={})
        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['duration'], ['Pole nie może być puste'])
        self.assertEqual(form.errors['name'], ['Pole nie może być puste'])

    def test_add_service_form_validation_for_wrong_duration(self):
        form = AddServiceForm(data={'duration':'ffff', 'name':'usługa'})
        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['duration'], ['Nie kombinuj!'])

    def test_add_service_form_validation_for_long_names(self):
        name = "Muvaffakiyetsizleştiricilestiriveremeyebileceklerimizdenmissinizcesine"
        form = AddServiceForm(data={'duration':'12', 'name':name})
        self.assertFalse(form.is_valid())
        self.assertEqual(form.errors['name'], ['Nazwa jest za długa'])


        # TODO Formularz powinien walidować duplikaty. Teraz robi to widok
        #  def test_add_service_form_validation_for_duplicate(self):


class PanelSettingsViewTests(BaseTest):
    def test_service_form_display(self):
        self.authorize_user()
        response = self.client.get('/ustawienia/')
        self.assertContains(response, 'add-service-form')

    def test_service_form_display_errors(self):
        name = "Muvaffakiyetsizleştiricilestiriveremeyebileceklerimizdenmissinizcesine"
        self.authorize_user()
        Service.objects.create(user=self.user, duration=timedelta(hours=1), name='duplikat')
        data_results = [{'data': {'duration': '00:15', 'name': ''}, 'message': 'Pole nie może być puste'},
                        {'data': {'duration': '00:00', 'name': 'usługa'}, 'message': 'Ustaw czas'},
                        {'data': {'duration': '01:45', 'name': name}, 'message': "Nazwa jest za długa"},
                        {'data': {'duration': '05:45', 'name': 'duplikat'}, 'message': 'Usługa o tej nazwie już istnieje'},]
        for data_result in data_results:
            response = self.client.post('/ustawienia/', data=data_result['data'])

            self.assertContains(response, data_result['message'])

    def test_service_empty_list_not_display(self):
        self.authorize_user()
        response = self.client.get('/ustawienia/')
        self.assertContains(response, "Nie masz jeszcze żadnych usług")

    def test_service_list_display(self):
        self.authorize_user()
        #TODO usługa dodana
        response = self.client.post('/ustawienia/', data={'duration': '00:15', 'name': 'usługa'})
        self.assertContains(response, "Usługa usługa dodana.")
        self.assertContains(response, "<td>usługa</td>")
        self.assertContains(response, "<td>0:00:15</td>")
        self.assertContains(response, "<td>False</td>")
