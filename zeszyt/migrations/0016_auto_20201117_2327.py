# Generated by Django 3.1.3 on 2020-11-17 22:27

from django.db import migrations


class Migration(migrations.Migration):

    dependencies = [
        ('zeszyt', '0015_auto_20201117_2147'),
    ]

    operations = [
        migrations.RenameField(
            model_name='worktime',
            old_name='stop_time',
            new_name='end_time',
        ),
    ]