import random
s = 10
b_tmp = []
for i in range(s+1):
    b_tmp.append(random.uniform(0, 1))

b_tmp.sort(reverse=False)

b_new = []
for i in range(s):
    b_new.append(b_tmp[i+1]-b_tmp[i])
