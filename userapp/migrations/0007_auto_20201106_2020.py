# Generated by Django 3.1.3 on 2020-11-06 19:20

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('userapp', '0006_auto_20201106_2010'),
    ]

    operations = [
        migrations.AddField(
            model_name='client',
            name='next_pin',
            field=models.CharField(default='', max_length=4),
        ),
        migrations.AddField(
            model_name='client',
            name='pin',
            field=models.CharField(default='', max_length=4),
        ),
    ]