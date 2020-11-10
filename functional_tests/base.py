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
        User.objects.create_user(username='user2', password='pass2')
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
    def find_input_boxes_for_login(self):
        self.inputbox_name = self.browser.find_element_by_id('id_username')
        self.inputbox_password = self.browser.find_element_by_name('password')
        self.login_button = self.browser.find_element_by_name('login')

    def find_input_boxes_for_client_login(self):
        self.input_phone = self.browser.find_element_by_name('phone_number')
        self.input_pin = self.browser.find_element_by_name('pin')
        self.submit_btn = self.browser.find_element_by_name('submit')

    def send_form(self, field):
        for field_name, field_value in field.items():
            self.browser.find_element_by_name(field_name).clear()
            self.browser.find_element_by_name(field_name).send_keys(field_value)
        self.browser.find_element_by_name('submit').click()