import matplotlib.pyplot as plt
filename = 'controlData.txt'
X,Y = [],[]
Xref,Yref = [],[]
ephi = []
ed = []
omegaee=[]
omega=[]
with open(filename, 'r') as f:#1
    lines = f.readlines()#2
    for line in lines:#3
        value = [float(s) for s in line.split()]#4
        X.append(value[2])#5
        Y.append(value[3])
        Xref.append(value[0])
        Yref.append(value[1])
        ephi.append(value[5])          
        ed.append(value[4])
        #omegaee.append(value[14])
        #omega.append(value[15])

#print(X)
#print(Y)


plt.plot(X, Y, 'r')
plt.plot(Xref, Yref, 'g')
plt.show()

plt.plot(ephi,'r')
plt.show()

plt.plot(ed, 'g')
plt.show()

# plt.plot(omega)
# plt.show()

# plt.plot(omegaee)
# plt.show()
