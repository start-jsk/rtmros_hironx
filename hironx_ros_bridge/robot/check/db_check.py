#!/usr/bin/env python

import requests
def db_check(msg):
    url = 'https://docs.google.com/forms/d/1FuvFIWQui1dawi9OtPgEo4Q6V729hkhjfRetJKtId3Q/formResponse'
    form_data = {'entry.1561122805':msg}
    user_agent = {'Referer':'https://docs.google.com/forms/d/1FuvFIWQui1dawi9OtPgEo4Q6V729hkhjfRetJKtId3Q/viewform','User-Agent': "Mozilla/5.0 (X11; Linux i686) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/28.0.1500.52 Safari/537.36"}

    try:
        r = requests.post(url, data=form_data, headers=user_agent)
    except Exception as e:
        print e
    if not r.ok:
        raise Exception("Could not find internet connection")
    print "\n* Saving DB..\n"
    return True



