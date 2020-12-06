from django.contrib.auth.models import User
from django.core.exceptions import ValidationError
from django.db.utils import IntegrityError
from django.test import TestCase
from unittest import skip
from ..models import Client, Service, Visit, WorkTime
from .base import BaseTest

class ClientsTests(BaseTest):

    """ Add clients """

    def test_client_model_save(self):
        user = self.create_user()
        client = self.clients['active_1']
        client = Client.objects.create(phone_number=client['phone_number'], name=client['name'], user=user)

        self.assertEqual(client, Client.objects.first())


    def test_errors_missing_required_fields(self):
        user = self.create_user()
        client_data = self.clients['active_1']
        clients = [Client(user=user, name=client_data['name']),
                   Client(user=user, phone_number=client_data['phone_number']),
                   Client(phone_number=client_data['phone_number'], name=client_data['name'])
        ]
        for client in clients:
            with self.assertRaises(ValidationError):
                client.full_clean()


    def test_errors_incorrect_fields(self):
        # TODO: Inne błędy też
        user = self.create_user()
        client = self.clients['active_1']

        with self.assertRaises(ValidationError):
            client = Client(phone_number=self.WRONG_DATA, name=client['name'], user=user)
            client.full_clean()


    def test_client_is_related_to_user(self):
        user = self.create_user()
        client = self.clients['active_1']
        client = Client(phone_number=client['phone_number'], name=client['name'])
        client.user = user
        client.save()

        self.assertIn(client, user.client_set.all())

    def test_errors_duplicate_numbers(self):
        user = self.create_user()
        client = self.clients['active_1']
        Client.objects.create(user=user, phone_number=client['phone_number'], name=client['name'])

        with self.assertRaises(ValidationError):
            client = Client(user=user, phone_number=client['phone_number'], name=client['name'])
            client.full_clean()


    def test_duplicate_numbers_for_different_users(self):
        """ Different users can add client with the same phone number """
        # TODO: Zorientowac się, jaki test tu zastosować
        user1 = self.create_user('active_1')
        user2 = self.create_user('active_2')
        client = self.clients['active_1']
        client_full = self.clients_full_data['active_1']
        Client.objects.create(user=user1, phone_number=client['phone_number'], name=client['name'])
        client = Client(user=user2, phone_number=client['phone_number'], pin=client_full['pin'], name=client['name'])
        client.full_clean()  # Nie powinien być zgłoszony


    """ Remove clients """


    def test_client_remove(self):
        user = self.create_user()
        client = self.clients['active_1']
        client = Client.objects.create(phone_number=client['phone_number'], name=client['name'] , user=user)
        client.delete()

        self.assertFalse(Client.objects.first())

    def test_get_remove_url(self):
        user = self.create_user()
        client = self.clients['active_1']

        client = Client.objects.create(phone_number=client['phone_number'], name=client['name'] , user=user)

        self.assertEqual(client.get_remove_url(), f'/klienci/usun/{client.id}/')

class ServiceTests(BaseTest):

    """ Add service tests """

    def test_service_model_save(self):
        user = self.create_user()
        service = self.services['short_1']
        service = Service.objects.create(duration=service['duration'], name=service['name'], user=user)

        self.assertEqual(service, Service.objects.first())


    @skip
    def test_errors_missing_required_fields(self):
        #TODO: Dla duration = None powinno się walidować - dla AttributeError działa, ale nie dla Validation_error
        user = self.create_user()
        service = self.services['short_1']

        services = [Service(name=service['name'], duration=service['duration']),
                    Service(user=user, duration=service['duration']),
                    Service(user=user, name=service['name'])
        ]
        for service in services:
            with self.assertRaises(ValidationError):
                service.full_clean()


    @skip #TODO: Walidacja dla niewłaściwego czasu
    def test_errors_incorrect_duration(self):
        user = self.create_user()
        service = self.services['short_1']

        services = [Service(user=user, name=service['name'], duration=self.DURATION_MIN),
                    Service(user=user, name=service['name'], duration=self.DURATION_MAX),
        ]
        for service in services:

            with self.assertRaises(ValidationError):
                service.full_clean()


    def test_service_is_related_to_user(self):
        user = self.create_user()
        service = self.services['short_1']
        service = Service(duration=service['duration'], name=service['name'])
        service.user = user
        service.save()

        self.assertIn(service, user.service_set.all())


    def test_errors_duplicate_service_name(self):
        user = self.create_user()
        service = self.services['short_1']
        Service.objects.create(user=user, duration=service['duration'], name=service['name'])

        with self.assertRaises(ValidationError):
            service = Service(user=user, duration=service['duration'], name=service['name'])
            service.full_clean()


    def test_duplicate_services_for_different_users(self):
        # TODO: Uspuełnij o prawidłowy test
        user1 = self.create_user('active_1')
        user2 = self.create_user('active_2')
        service = self.services['short_1']
        Service.objects.create(user=user1, duration=service['duration'], name=service['name'])
        service = Service(user=user2, duration=service['duration'], name=service['name'])
        service.full_clean()  # Nie powinien być zgłoszony


    def test_long_service_name(self):
        user = self.create_user()
        service = self.services['short_1']

        with self.assertRaises(ValidationError):
            service = Service(user=user, duration=service['duration'], name=self.LONG_STRING(61))
            service.full_clean()


    """ Remove service tests """

    def test_service_remove(self):
        user = self.create_user()
        service = self.create_service(user)
        service.delete()

        self.assertFalse(Service.objects.first())

    def test_get_remove_url(self):
        user = self.create_user()
        service = self.create_service(user)

        self.assertEqual(service.get_remove_url(), f'/ustawienia/usun_usluge/{service.id}/')

class UserTests(BaseTest):

    def test_user_saving(self):
        user = self.users['active_1']
        user = User.objects.create_user(username=user['username'], password=user['password'])

        self.assertEqual(user, User.objects.first())

    def test_create_work_time_for_user(self):
        user = self.users['active_1']
        user = User.objects.create_user(username=user['username'], password=user['password'])
        work_time = WorkTime.objects.get(user=user)
        self.assertTrue(work_time)

class VisitTests(BaseTest):

    def test_visit_saving(self):
        user = self.create_user()
        client = self.create_client(user)
        visit = self.visits['short_1']
        visit = Visit.objects.create(user=user, client=client, name=visit['name'], start=visit['start'], end=visit['end'])

        self.assertEqual(visit, Visit.objects.first())


    def test_visit_is_related_to_client_user(self):
        user = self.create_user()
        client = self.create_client(user)
        visit = self.visits['short_1']
        visit = Visit(name=visit['name'], start=visit['start'], end=visit['end'])
        visit.user = user
        visit.client = client
        visit.save()

        self.assertIn(visit, user.visit_set.all())
        self.assertIn(visit, client.visit_set.all())

    def test_errors_missing_relate_fields(self):
        user = self.create_user()
        client = self.create_client(user)
        visit = self.visits['short_1']

        visits = [Visit(user=user, client=client, name=visit['name'], start=visit['start']),
                  Visit(user=user, client=client, name=visit['name'], end=visit['end']),]

        for visit in visits:
            with self.assertRaises(AttributeError):
                visit.full_clean()

    def test_errors_missing_datetime_fields(self):
        user = self.create_user()
        client = self.create_client(user)
        visit = self.visits['short_1']

        visits = [Visit(user=user, name=visit['name'], start=visit['start'], end=visit['end']),
                  Visit(client=client, name=visit['name'], start=visit['start'], end=visit['end']),]

        for visit in visits:
            with self.assertRaises(ValueError):
                visit.full_clean()

    def test_validation_errors(self):
        user = self.create_user()
        client = self.create_client(user)
        visit = self.visits['short_1']
        visits = [Visit(user=user, client=client, start=visit['start'], end=visit['end']),
                  Visit(user=user, name=self.LONG_STRING(61), client=client, start=visit['start'], end=visit['end'])]

        for visit in visits:
            with self.assertRaises(ValidationError):
                visit.full_clean()

    def test_visit_overlaps_error(self):
        user = self.create_user()
        client = self.create_client(user)
        visit = self.visits['short_1']
        overlap_visits = [self.visits['overlap_1'], self.visits['overlap_2']]

        Visit.objects.create(user=user, client=client, name=visit['name'], start=visit['start'], end=visit['end'])
        for visit in overlap_visits:
            with self.assertRaises(ValidationError):
                visit = Visit(user=user, client=client, name=visit['name'], start=visit['start'], end=visit['end'])
                visit.full_clean()

    def test_visit_not_overlaps_error(self):
        user = self.create_user()
        client = self.create_client(user)
        visit = self.visits['short_1']
        not_overlap_visits = [self.visits['not_overlap_1'], self.visits['not_overlap_2']]

        Visit.objects.create(user=user, client=client, name=visit['name'], start=visit['start'], end=visit['end'])
        for visit in not_overlap_visits:
            #TODO: Jaki rodzaj testu tutaj?
            visit = Visit(user=user, client=client, name=visit['name'], start=visit['start'], end=visit['end'])
            visit.full_clean()

    def test_visit_overlaps_for_different_users(self):
        user_1 = self.create_user('active_1')
        user_2 = self.create_user('active_2')
        client_1 = self.create_client(user_1)
        client_2 = self.create_client(user_2)
        visit = self.visits['short_1']
        Visit.objects.create(user=user_1, client=client_1, name=visit['name'], start=visit['start'], end=visit['end'])

        # TODO: Jaki rodzaj testu tutaj?
        visit = Visit(user=user_2, client=client_2, name=visit['name'], start=visit['start'], end=visit['end'])
        visit.full_clean()


class WorkTimeTests(BaseTest):

    def test_worktime_edit(self):
        user = self.create_user()
        work_time = WorkTime.objects.get(user=user)
        work_time.end_time = "14:00"
        # TODO: Znajdź sposob na ten test
        work_time.full_clean()

    def test_work_hours_error(self):
        user = self.create_user()
        work_time = WorkTime.objects.get(user=user)
        work_time.end_time = "01:00"
        work_time.start_time = "02:00"
        with self.assertRaises(ValidationError):
            work_time.full_clean()

    def test_work_avaible_days_error(self):
        user = self.create_user()
        work_time = WorkTime.objects.get(user=user)
        work_time.earliest_visit = 15
        work_time.latest_visit = 10
        with self.assertRaises(ValidationError):
            work_time.full_clean()

    @skip
    def test_work_negative_days_error(self):
        user = self.create_user()
        work_time = WorkTime.objects.get(user=user)
        work_time.earliest_visit = -20
        work_time.latest_visit = -15
        with self.assertRaises(ValidationError):
            work_time.full_clean()