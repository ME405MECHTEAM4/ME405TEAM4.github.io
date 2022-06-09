

def decimalToBinary(n):
    number = bin(n)[2:]
    print('actual val',n)
    print('bit number',number)
    print(number)
    if len(number) >8:
        maxB = 256
        first8 = number[len(number)-8:len(number)]
        second8 = number[:len(number)-8]
        ValF8 = int(first8,2)
        TX[3] = ValF8
        TX[2] = int((n-ValF8)/maxB)
        TX[1] =0
    else:
        TX[3] = n
    

def output():
    for idx,byte in enumerate(TX): print(f"b{idx}: {byte:#010b} {byte:#04x}")
 


if __name__ == '__main__':
    dec_val  = 0
    TX = bytearray(4)
    TX[0] = 0b00110010
    #TX[3] = number
    # decimal value
    x_array = [0, 0,  0, 7.5,7.5,7.5,7.5,0,0,7.5,7.5, 0,  0, 7.5,7.5,0,0]
    y_array = [0,7.5,7.5,7.5,7.5, 0,  0, 0,0,7.5,7.5,7.5,7.5, 0,  0, 0,0]
    commands = ['PU','PD','PD','PD','PD','PD','PD','PD','PD','PD','PD','PU','PU','PD','PD','PU','PU']
    
    
    for i in range(len(x_array)):
        if x_array[i] == 0 and y_array[i] ==0: 
            dec_val += 0
            decimalToBinary(dec_val)
            output()
        elif x_array[i] == 7.5:
            dec_val += 128
            decimalToBinary(dec_val)
            output()
    


















