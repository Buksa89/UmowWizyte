# Generated by Django 3.1.3 on 2020-12-03 21:34

from django.db import migrations


class Migration(migrations.Migration):

    dependencies = [
        ('userapp', '0021_auto_20201203_2200'),
    ]

    operations = [
        migrations.RenameField(
            model_name='visit',
            old_name='stop',
            new_name='end',
        ),
    ]
