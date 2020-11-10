from .base import BaseTest

class TestTemplate(BaseTest):

    def test_user_panel_template(self):
        self.authorize_user()
        response = self.client.get('/panel/')
        self.assertEqual(response.status_code, 200)
        self.assertTemplateUsed(response, 'panel/panel.html')