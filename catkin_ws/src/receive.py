#!/usr/bin/python
# -*- coding: utf-8 -*-

import requests
aaa = input("Enter first data------>")
bbb = input("Enter second data------>")
sendData = {'aaa':aaa,'bbb':bbb,'token':'7XWE32L52T03DH61'}
r = requests.get("http://coursesrv.nutn.edu.tw/S10582015/receive.php",params=sendData)
