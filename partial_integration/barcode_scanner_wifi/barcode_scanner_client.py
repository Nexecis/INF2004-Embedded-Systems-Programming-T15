import socket
import sys
import platform

def get_char():
    if platform.system() == 'Windows':
        import msvcrt
        return msvcrt.getch().decode('utf-8')
    else:
        import tty
        import sys

        fileno = sys.stdin.fileno()
        old_settings = tty.tcgetattr(fileno)
        try:
            tty.setraw(fileno)
            char = sys.stdin.read(1)
        finally:
            tty.tcsetattr(fileno, tty.TCSADRAIN, old_settings)
        return char
    
# Check server ip address set
if len(sys.argv) < 2:
    raise RuntimeError('pass IP address of the server')

# Put IP into SERVER_ADDR   192.168.29.109
SERVER_ADDR = sys.argv[1]

# These constants should match the server
BUF_SIZE = 1
SERVER_PORT = 4242
TEST_ITERATIONS = 1

# Open socket to the server
sock = socket.socket()
addr = (SERVER_ADDR, SERVER_PORT)
sock.connect(addr)

# Repeat test for a number of iterations
while(True):

    # Read BUF_SIZE bytes from the server
    total_size = BUF_SIZE
    read_buf = b''
    while total_size > 0:
        buf = sock.recv(BUF_SIZE)
        print('read %d bytes from server' % len(buf))
        total_size -= len(buf)
        read_buf += buf

        # Print the received data
    print('Received data:', read_buf)

        # Check size of data received
    if len(read_buf) != BUF_SIZE:
        raise RuntimeError('wrong amount of data read %d', len(read_buf))

    # Send the data back to the server
    # send_buf = b'JINGYI'
    # write_len = sock.send(send_buf)
    # print('written %d bytes to server' % write_len)
    # if write_len != BUF_SIZE:
    #     raise RuntimeError('wrong amount of data written')

# while True:
#     user_char = get_char()
#     send_buf = user_char.encode('utf-8')
#     write_len = sock.send(send_buf)
#     print('written %d bytes to server' % write_len)
#     if write_len != BUF_SIZE:
#         raise RuntimeError('wrong amount of data written')
#     if(send_buf == b'X'):
#         break

sock.close()