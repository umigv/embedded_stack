import time


f = open("demofile2.txt", "a")
for i in range(200):
    j = i*i
    f.write(str(i)+", " +str(j)+ "\n")
    f.flush()
    time.sleep(2)