import os
 
def disk_usage(path):
    """Return disk usage statistics about the given path.

    Returned valus is a named tuple with attributes 'total', 'used' and
    'free', which are the amount of total, used and free space, in bytes.
    """
    st = os.statvfs(path)
    free = st.f_bavail * st.f_frsize
    total = st.f_blocks * st.f_frsize
    used = (st.f_blocks - st.f_bfree) * st.f_frsize
    return (total, used, free)

def qnx_hdd_check():
    ret = True
    s = disk_usage('/opt')
    print "  Check HDD Space .. ",
    print "  Total %d MB, Used %d MB, Free %d MB\t\t\t" % ( s[0]/(1024*1024), s[1]/(1024*1024), s[2]/(1024*1024)), 
    if s[2] < 4 * 1024 * 1024 * 1024: # less than 4G
        ret = False
    print ret
    return ret

