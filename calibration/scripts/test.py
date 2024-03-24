from datetime import datetime
from datetime import timedelta

date = datetime(2024, 3, 24, 21, 3, 58, 300)
date = date + timedelta(microseconds=30)
ts = date.timestamp()
a = "1234"
print(float(a))