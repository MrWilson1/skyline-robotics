#!/usr/bin/env python3.2
#
# CGI script for contact form
# Sunday, May 07 2011
# Written by Michael Lee.

import os
import cgi
import cgitb

cgitb.enable()

import Access


### Create instance of form ###
form = cgi.FieldStorage()



### Get data. ###
if type(form.getvalue('name')) == str:
    name = form.getvalue('name')
else:
    name = "[None provided]"
    
if type(form.getvalue('email')) == str:
    email = form.getvalue('email')
else:
    email = "[None provided]"

if type(form.getvalue('subject')) == str:
    subject = form.getvalue('subject')
else:
    subject = "[None provided]"

if type(form.getvalue('message')) == str:
    message = form.getvalue('message')
else:
    message = "[No Message]"

about = form.getvalue('about')



### Prepare email text ###
message_template = "TIME:    {0}\n" \
                   "NAME:    {1}\n" \
                   "EMAIL:   {2}\n" \
                   "SUBJECT: {3}\n" \
                   "\n" \
                   "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" \
                   "\n" \
                   "MESSAGE:\n" \
                   "{4}\n" \
                   "\n" \
                   "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" \
                   "\n" \
                   "INFO DUMP (bested viewed in monospace): \n"\
                   "{5}"

message = message_template.format(Access.current_time(),
                                  name,
                                  email,
                                  subject,
                                  message,
                                  Access.list_params(str))



### Send email ###
Access.send_email(message, subject=about)



### Redirect to landing page. ###
# Todo: Replace with AJAX style thingy?
print("Content-Type: text/html")
print("Location: /feedback.html\n\n")



