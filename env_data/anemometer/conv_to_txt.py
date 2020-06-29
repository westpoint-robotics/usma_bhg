import sys

def conv(filename01, filename02):
    fin = open(filename01, mode='r')
    fout = open(filename02, "wt")

    lines = fin.readlines()

    count = 0
    for i in lines:
        if count < len(lines)-1:
            i = i.replace('\x00','')
            i = i.split()
            s = ','.join([i[0],i[1],i[2],i[4],i[5]])
            s = s + '\n'
            fout.write(s)
        count = count+1

    fin.close()
    fout.close()

    print('Convert Completed!\nOutput File Name >> %s' %filename02)


try:
    filein = sys.argv[1]
    fileout = sys.argv[2]
except:
    print('You failed to provide file name as input on the command line!\n ex) $ python conv_to_txt.py anemometer_input.txt output.txt')
    sys.exit(1)
conv(filein,fileout)
