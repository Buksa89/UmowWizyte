# Generated by Django 3.1.3 on 2020-11-06 19:10

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('zeszyt', '0005_auto_20201106_1950'),
    ]

    operations = [
        migrations.AddField(
            model_name='client',
            name='description',
            field=models.CharField(blank=True, default='', max_length=200),
        ),
        migrations.AlterField(
            model_name='client',
            name='surname',
            field=models.CharField(blank=True, default='', max_length=40),
        ),
    ]
