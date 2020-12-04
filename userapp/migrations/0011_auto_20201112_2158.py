# Generated by Django 3.1.3 on 2020-11-12 20:58

from django.conf import settings
from django.db import migrations


class Migration(migrations.Migration):

    dependencies = [
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
        ('userapp', '0010_service'),
    ]

    operations = [
        migrations.AlterUniqueTogether(
            name='service',
            unique_together={('user', 'name')},
        ),
    ]