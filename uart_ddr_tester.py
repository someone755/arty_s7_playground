# Serial port of Arty-S50 board
serialPort = 'COM5'
# Baud rate set in Artix bitstream
baudRate = 12000000
# Size of data to write to DDR3, in bytes
# Must be multiple of 8
sizeOfData = 640

import serial
import os
import hashlib
from time import sleep
def genDataTx():
    print('I: Creating new blob of size', sizeOfData)
    dataTxL = os.urandom(sizeOfData)
    open('dataTx.bin', 'wb').write(dataTxL)

# Open serial port
ser = serial.Serial(serialPort, baudRate, timeout=0.5)

# Make sure DDR is empty by flushing with EOF word
print('I: Flushing DDR with EOF word...')
for i in range(0,16):
    ser.write(b'\x66')
    ackByte = ser.read()
    if (ackByte != b'\x8a') and (ackByte != b''):
        print('I: DDR now empty. ACK =', ackByte)
        break
sleep(1)

# Close and reopen serial port; this drops the current TX buffer
ser.close()
ser = serial.Serial(serialPort, baudRate, timeout=0.5)

# Check data generation
if sizeOfData % 8 :
    print('E: Data size must be multiple of 8!')
    quit()

# Check if file exists
if os.path.exists('dataTx.bin'):
    if os.path.getsize('dataTx.bin') == sizeOfData:
        print('I: Found blob size of size', sizeOfData)
    else:
        genDataTx()
else:
    genDataTx()

dataTx = open('dataTx.bin','rb').read()
# Generate md5 hash
dataTxHash = hashlib.md5(dataTx).hexdigest()

# Send 8 bytes, wait for ack
print('I: TX data blob...')
for i in range (0, sizeOfData, 8):
    ser.write(dataTx[i:i+8])
    ackByte = ser.read()
    # print('{} I: ACK byte is'.format(i/8), ackByte)#, end =" ")
    if ackByte != b'\x8a':
        print(i,"E: RX byte {} != b'x8a'".format(ackByte))
        quit()

# Send 64-bit EOF signal
print('I: TX EOF word..')
for i in range (0,8):
    ser.write(b'\x66')

# Receive data of len(sizeOfData)
print('I: Receiving data from DDR...')
dataRx = ser.read(sizeOfData)
open('dataRx.bin', 'wb').write(dataRx)

# Generate MD5 for RX data
print('I: Generating RX data MD5...')
dataRxHash = hashlib.md5(dataRx).hexdigest()

# Final MD5 comparison
if dataRxHash == dataTxHash :
    print('I: Success! File hash match')
else:
    print('E: File hash mismatch!')
    print(len(dataRx), len(dataTx))
    
ser.close()
