#!/usr/bin/evn python3.2
#
# Library for accessing log files.

import os
import time
import smtplib
import csv



def _read_ini_file(path):
    with open(path, "r") as file:
        dialect = csv.Sniffer().sniff(file.read())
        file.seek(0)
        csv_object = csv.reader(file, dialect)
        parsed = {}
        for row in csv_object:
            if row:
                parsed[row[0].strip()] = row[1].strip()
        return parsed


def _read_email(path):
    # Open CSV reader, read values.
    _email_parsed = _read_ini_file(path)
        
    # Change all recipients to a list.
    recipients = _email_parsed['recipients']
    r_list = recipients.split(',')
    for (num, item) in enumerate(r_list):
        r_list[num] = item.strip()
    _email_parsed['recipients'] = r_list
    return _email_parsed


def send_email(message,
               from_=None,
               recipients=None,
               subject='No Subject'):
    if not(from_):
        from_ = _email_parsed['sender']
    if not(recipients):
        recipients = _email_parsed['recipients']
        
    template = "From: <{0}>\n" \
               "To: <{1}>\n" \
               "Subject: {2}\n" \
               "\n" \
               "{3}\n"
    message = template.format(from_, recipients, subject, message)
    
    smtpObj = smtplib.SMTP(_email_parsed['mail_server'])
    smtpObj.login(_email_parsed['username'], _email_parsed['password'])
    smtpObj.sendmail(_email_parsed['sender'], recipients, message)
    return

    
def current_time():
    return time.strftime("%Y-%m-%d %H:%M:%S (%A)")


def list_params(format_type=dict):
    params_list = {}
    for param in os.environ.keys():
        params_list[param] = os.environ[param]
        
    if format_type == str:
        spacer = len(max(params_list.keys(), key=len)) + 2
        temp = []
        for param in params_list.keys():
            temp.append(param)
            for i in range(spacer - len(param)):
                temp.append(" ")
            temp.append(":")
            temp.append(os.environ[param])
            temp.append("\n")
        return ''.join(temp)
    else: # Intentionally not strict.
        return params_list
    return



_email_parsed = _read_email("/home/sparta/__private__/email_passes.txt")
    
