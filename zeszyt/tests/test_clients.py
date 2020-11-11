from unittest import skip
from .base import BaseTest


class PanelClientsTemplateTests(BaseTest):

    def test_user_clients_template(self):
        self.authorize_user()
        response = self.client.get('/klienci/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'panel/clients.html')
