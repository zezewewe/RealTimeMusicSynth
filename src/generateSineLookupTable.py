import math

sineLookUpTable=[]

for phase in range(-128,128):
    sineLookUpTable.append(round(127*math.sin((phase+128)/255*2*math.pi)))
