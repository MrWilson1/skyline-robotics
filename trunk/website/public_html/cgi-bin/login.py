#!/usr/bin/env python3.2
#
# CGI script for logging in admins.
# Uses cookies for authentication.
# Sunday, May 22 2011
# Written by Michael Lee.

import sys
import os

import cgi
import cgitb

cgitb.enable()


### Grab input ###
form = cgi.FieldStorage()


if type(form.getvalue('username')) == str:
    username = form.getvalue('username')
else:
    username = None
    
if type(form.getvalue('password')) == str:
    password = form.getvalue('password')
else:
    username = None

if (username == None) or (password == None):
    print('Content-type: text/html\n')
    print('Location: /errors/403.html')

