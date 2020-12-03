import math


motorsConsumption = 12   #Watts

Imp               = 0.6  #Amperes
Vmp               = 16.5 #Volts
Isc               = 1.2  #Amperes
Voc               = 22.1 #Volts
Pmax              = 10   #Watts
alfa              = 0.039 * Imp
beta              =-0.307 * Vmp
S                 = 800
Sref              = 1000
Tref              = 24.72829
Rs                = 33.333333
V                 = Vmp
tempAtual         = 25


def Emotors(secondsActived):
    return motorsConsumption*secondsActived

def C1():
    
    return (1 - (Imp/Isc)) * math.exp(-Vmp/(C2()*Voc))

def C2():
    
    return ((Vmp/Voc)-1)/math.log(1-(Imp/Isc))

def deltaTemp(T):
    return T - Tref

def deltaI():
    return alfa * (S/Sref) * deltaTemp(tempAtual) + (S/Sref)*Isc

def deltaV():
    return ((-1*beta*deltaTemp(tempAtual)) - (Rs*deltaI()))

def currentPanel():
    return Isc * (1 - C1()*(math.exp((V-deltaV())/(C2()*Voc))-1))+deltaI() 




def main():
    print("Hello World!")
    print(currentPanel())





if __name__ == "__main__":
    main()