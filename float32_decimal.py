import numpy as np
import ctypes

def float2decimal(num):
    # this function transforms float32 to string type and
    # then split it into 4 Bytes of a float32 data
    binNum = bin(ctypes.c_uint.from_buffer(ctypes.c_float(num)).value)[2:]
    mantissa = "1" + binNum[-23:]
    mantInt = int(mantissa,2)/2**23
    base = int(binNum[-31:-23],2)-127
    sign = 1-2*("1"==binNum[-32:-31].rjust(1,"0"))

    print("bits: " + binNum.rjust(32,"0"))
    print("sig (bin): " + mantissa.rjust(24))
    print("sig (float): " + str(mantInt))
    print("base:" + str(base))
    print("sign:" + str(sign))
    print("recreate:" + str(sign*mantInt*(2**base)))
    
    # split the data into 4 Bytes and then transform it to decimal
    B = str(binNum.rjust(32,"0"))[0:8]
    G = str(binNum.rjust(32,"0"))[8:16]
    R = str(binNum.rjust(32,"0"))[16:24]
    alPha = str(binNum.rjust(32,"0"))[24:32]

    # changing from binary int to decimal
    B_r = bin2decimal(int(B))
    G_r = bin2decimal(int(G))
    R_r = bin2decimal(int(R))
    alPha_r = bin2decimal(int(alPha))
    
    return B_r,G_r,R_r,alPha_r

def bin2decimal(binary):
    # binary element to decimal
    sum = 0
    counter = 0
    while binary > 0:
        y = binary % 10
        binary = binary // 10
        sum = sum + y * (2**counter)
        counter += 1
    #print("Your result in decimal is: ", sum)
    return sum

a,b,c,d= float2decimal(-2.835949e+38)
print('................')
print(type(a))
print(a)

print('................')
print(type(b))
print(b)

print('................')
print(type(c))
print(c)

print('................')
print(type(d))
print(d)


