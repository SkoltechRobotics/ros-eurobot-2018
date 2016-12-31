import datetime

string_time = "2017-01-01 00:00:00"
try:
    f = open("last_time", "r")
    for line in f:
        string_time = line
    t = datetime.datetime.strptime(string_time, "%Y-%m-%d %H:%M:%S")
    t2 = t + datetime.timedelta(minutes=30)
    f.close()
    string_time = datetime.datetime.strftime(t2, "%Y-%m-%d %H:%M:%S")
except IOError:
    string_time = "2017-01-01 00:00:00"

f = open("last_time", "w")
print string_time
f.write(string_time)
f.close()
