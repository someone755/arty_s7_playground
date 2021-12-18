#    Arty-S50 UART-DDR3 tester    #

if True: ## Global script settings
    # Serial port of Arty-S50 board
    serialPort = 'COM5'
    # Baud rate set in Artix bitstream
    baudRate = 6000000
    # Size of data to write to DDR3, in bytes
    # Must be multiple of 16, up to module size
    # of 2 Gbit or 2**28 = 268435456 bytes
    sizeOfData = 2**14#2**28
    # Whether to wait for ACK upon TX
    waitForAck = False
    # Whether to flush DDR with EOF word before TX
    flushDDR = True
    # Whether to set starting address
    setAddr = True
    # Write starting address, max 2**27
    # Must be multiple of 8 (BL8)
    startAddr = 2**10#2**27/16

if True: ## Library imports
    import serial
    import os
    import hashlib
    import time

if True: ## Function declarations
    def genDataTx():
        print('[{}] I: Creating new blob of size'.format(time.time()-t), sizeOfData)
        dataTxL = os.urandom(sizeOfData)
        open('dataTx.bin', 'wb').write(dataTxL)
    
# Save script start time
t = time.time()

if True: ## Check variable sizes
    if sizeOfData % 16 :
        print('[{}] E: Data size must be multiple of 16!'.format(time.time()-t))
        quit()
    if startAddr % 8 :
        print('[{}] E: Address offset must be multiple of 8!'.format(time.time()-t))
        quit()

if True: ## Look for data blob on disk, generate if not found
    # Check if file exists
    if os.path.exists('dataTx.bin'):
        if os.path.getsize('dataTx.bin') == sizeOfData:
            print('[{}] I: Found blob of size'.format(time.time()-t), sizeOfData)
        else:
            genDataTx()
    else:
        genDataTx()

if True: ## Setup TX data and hash
    # Read dataTx from binary blob
    dataTx = open('dataTx.bin','rb').read()
    # Generate md5 hash
    dataTxHash = hashlib.md5(dataTx).hexdigest()

if flushDDR: ## Write up to 32 0xFF bytes (16*0xFF => RD op)
    # Open serial port
    ser = serial.Serial(serialPort, baudRate, timeout=0.1)
    ser.set_buffer_size(rx_size = sizeOfData, tx_size = sizeOfData)

    # Make sure DDR is empty by flushing with EOF word
    print('[{}] I: Flushing DDR with EOF word...'.format(time.time()-t))
    for i in range(0,33):
        ser.write(b'\x66')
        ackByte = ser.read()
        if (ackByte != b'\x8a') and (ackByte != b''):
            print('[{}] I: DDR now empty. ACK ='.format(time.time()-t), ackByte)
            break
    if ackByte == b'':
        print('[{}] E: No response from FPGA!'.format(time.time()-t))
        quit()
    time.sleep(1)

    # Close serial port
    ser.close()
    
if True: ## (Re-) Open serial port (reopen drops current TX buffer)
    ser = serial.Serial(serialPort, baudRate, timeout=5)
    ser.set_buffer_size(rx_size = sizeOfData, tx_size = sizeOfData)

if setAddr: ## Set starting addr, [127:64] => 0xaa, [27:0] => addr_start
    for i in range (0,12):
        ser.write(b'\xaa')
    ser.write(int(startAddr).to_bytes(4, 'big'))

if True: ## Send data
    print('[{}] I: TX data blob...'.format(time.time()-t))
    time1 = time.time()
    if (waitForAck): ## 16 bytes at a time
        i=int(0)
        ser.write(dataTx[i:i+16])
        while (i < sizeOfData): #in range (0, sizeOfData, 16):
            print(i/2, dataTx[i:i+16].hex())
            ackByte = ser.read(1)
            if ackByte == b'\x8a':
                i = i+16
                ser.write(dataTx[i:i+16])
            elif ackByte == b'\x88':
                print(i/2,"[{}] E: RX byte b'x8a' != ".format(time.time()-t), ackByte)
                time.sleep(1)
            elif ackByte == b'':
                print(i/2,"[{}] E: No response from FPGA!".format(time.time()-t))
                quit()
            
    else: ## All at once
        ser.write(dataTx)
        
    time2 = time.time()
    if (time2-time1) != 0:
        print('[{}] I: TX effective rate:'.format(time2-t),sizeOfData*8/(time2-time1)/1000,'kbit/s')
    time.sleep(2)

if True: ## Close and reopen serial port; this drops the current TX buffer
    rxTimeout = sizeOfData*11/baudRate
    ser.close()
    ser = serial.Serial(serialPort, baudRate, timeout=rxTimeout)
    ser.set_buffer_size(rx_size = sizeOfData, tx_size = sizeOfData)

if True: ## Send 128-bit EOF signal
    print('[{}] I: TX EOF word..'.format(time.time()-t))
    for i in range (0,16):
        ser.write(b'\x66')
    time.sleep(0.1)

if True: ## Receive data of len(sizeOfData)
    print('[{}] I: Receiving data from DDR...'.format(time.time()-t))
    time1 = time.time()
    dataRx = ser.read(sizeOfData)
    time2 = time.time()
    if (time2-time1) != 0:
        print('[{}] I: RX effective rate:'.format(time2-t),len(dataRx)*8/(time2-time1)/1000,'kbit/s')
    open('dataRx.bin', 'wb').write(dataRx)

# Generate MD5 for RX data
print('[{}] I: Generating RX data MD5...'.format(time.time()-t))
dataRxHash = hashlib.md5(dataRx).hexdigest()

if True: # Compare TX and RX data hashes
    if dataRxHash == dataTxHash :
        print('[{}] I: Success! File hash match!'.format(time.time()-t))
    else:
        print('[{}] E: File hash mismatch!'.format(time.time()-t))
        print("RX:", len(dataRx), "TX:", len(dataTx))
    
ser.close() # Close port for another run/program
