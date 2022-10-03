dictionary = {}
for i in range(2):
    dictionary[i] = dict(Timer = str("timer" + str(i)), Flag = True)
print(dictionary[i]['Timer'])