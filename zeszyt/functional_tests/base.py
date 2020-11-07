from django.contrib.auth.models import User
from django.contrib.staticfiles.testing import StaticLiveServerTestCase
from selenium import webdriver
from selenium.common.exceptions import WebDriverException
import time
MAX_WAIT = 3

class FunctionalTest(StaticLiveServerTestCase):
    def setUp(self):
        self.browser = webdriver.Firefox()
        User.objects.create_user(username='user_not_active', password='pass_not_active', is_active=False)
        User.objects.create_user(username='user', password='pass')
        self.browser.get(self.live_server_url+'/panel')
    def tearDown(self):
        self.browser.quit()
    def wait_for(self, fn):
        start_time = time.time()
        while True:
            try:
                return fn()
            except (AssertionError, WebDriverException) as e:
                if time.time() - start_time > MAX_WAIT:
                    raise e
                time.sleep(0.5)