# Serial port of Arty-S50 board
serialPort = 'COM5'
# Baud rate set in Artix bitstream
baudRate = 12000000
# Size of data to write to DDR3, in bytes
# Must be multiple of 8, up to 2**28 = 268435456
sizeOfData = 2**15

import serial
import os
import hashlib
import time
def genDataTx():
    print('[{}] I: Creating new blob of size'.format(time.time()-t), sizeOfData)
    dataTxL = os.urandom(sizeOfData)
    open('dataTx.bin', 'wb').write(dataTxL)
t = time.time()

# Open serial port
ser = serial.Serial(serialPort, baudRate, timeout=0.5)

# Make sure DDR is empty by flushing with EOF word
print('[{}] I: Flushing DDR with EOF word...'.format(time.time()-t))
for i in range(0,16):
    ser.write(b'\x66')
    ackByte = ser.read()
    if (ackByte != b'\x8a') and (ackByte != b''):
        print('[{}] I: DDR now empty. ACK ='.format(time.time()-t), ackByte)
        break
time.sleep(1)

# Close and reopen serial port; this drops the current TX buffer
ser.close()
ser = serial.Serial(serialPort, baudRate)

# Check data generation
if sizeOfData % 8 :
    print('[{}] E: Data size must be multiple of 8!'.format(time.time()-t))
    quit()

# Check if file exists
if os.path.exists('dataTx.bin'):
    if os.path.getsize('dataTx.bin') == sizeOfData:
        print('[{}] I: Found blob of size'.format(time.time()-t), sizeOfData)
    else:
        genDataTx()
else:
    genDataTx()

# Read dataTx from binary blob
dataTx = open('dataTx.bin','rb').read()
# Generate md5 hash
dataTxHash = hashlib.md5(dataTx).hexdigest()

# Send 8 bytes, wait for ack
time1 = time.time()
print('[{}] I: TX data blob...'.format(time1-t))
for i in range (0, sizeOfData, 8):
    ser.write(dataTx[i:i+8])
    #ackByte = ser.read()
    # print('{} I: ACK byte is'.format(i/8), ackByte)#, end =" ")
    if i%2**3 == 0:
        ackByte = ser.read()
        if ackByte != b'\x8a':
            print(i,"[{}] E: RX byte b'x8a' != ".format(time.time()-t), ackByte)
            quit()
time2 = time.time()
print('[{}] I: TX effective rate:'.format(time2-t),sizeOfData*8/(time2-time1)/1000,'kbit/s')

ser.close()
ser = serial.Serial(serialPort, baudRate)

# Send 64-bit EOF signal
print('[{}] I: TX EOF word..'.format(time.time()-t))
for i in range (0,8):
    ser.write(b'\x66')

# Receive data of len(sizeOfData)
print('[{}] I: Receiving data from DDR...'.format(time.time()-t))
time1 = time.time()
dataRx = ser.read(sizeOfData)
time2 = time.time()
print('[{}] I: RX effective rate:'.format(time2-t),len(dataRx)*8/(time2-time1)/1000,'kbit/s')
open('dataRx.bin', 'wb').write(dataRx)

# Generate MD5 for RX data
print('[{}] I: Generating RX data MD5...'.format(time.time()-t))
dataRxHash = hashlib.md5(dataRx).hexdigest()

# Final MD5 comparison
if dataRxHash == dataTxHash :
    print('[{}] I: Success! File hash match!'.format(time.time()-t))
else:
    print('[{}] E: File hash mismatch!'.format(time.time()-t))
    print(len(dataRx), len(dataTx))
    
ser.close()