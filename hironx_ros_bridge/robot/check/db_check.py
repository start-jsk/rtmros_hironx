#!/usr/bin/env python

def db_check(msg):

    try:
        import requests
    except ImportError, e:
        print str(e), ", skipping db_check."
        # Since failure of db_check does not mean the installability checking failed,
        # return true here so as not to effect the other checking steps.
        return True

    url = 'https://docs.google.com/forms/d/1FuvFIWQui1dawi9OtPgEo4Q6V729hkhjfRetJKtId3Q/formResponse'
    form_data = {'entry.1561122805':msg}
    user_agent = {'Referer':'https://docs.google.com/forms/d/1FuvFIWQui1dawi9OtPgEo4Q6V729hkhjfRetJKtId3Q/viewform','User-Agent': "Mozilla/5.0 (X11; Linux i686) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/28.0.1500.52 Safari/537.36"}

    print "\n* Saving DB..\n"
    try:
        r = requests.post(url, data=form_data, headers=user_agent)
    # Using Python 2.5 style. See http://stackoverflow.com/questions/5119751/in-python-whats-the-difference-between-except-exception-as-e-and-except-exc
    except Exception, e:
        print "".join(map(lambda s: hex(s), [ord(s) for s in e.message.message]))
        return False
    if not r.ok:
        print "  ... Could not find internet connection"
    return r.ok



