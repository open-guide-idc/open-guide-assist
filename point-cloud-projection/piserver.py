import socket
import RPi.GPIO as GPIO

#setup PI
GPIO.setmode(GPIO.BOARD)
#motor1
GPIO.setup(8,GPIO.OUT)
pwm = GPIO.PWM(8, 100)
pwm.start(0)

HOST_IP = '155.41.35.46'
PORT = 2000
BUFFER_SIZE = 20

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Socket created')

try:
    s.bind((HOST_IP, PORT))
except socket.error:
    print('Bind failed')

s.listen(1)
print('Socket awaiting messages')
conn, addr = s.accept()
print('Connection address:', addr)
while True:
    data = conn.recv(BUFFER_SIZE)
    if data == '1':
        pwm.ChangeDutyCycle(100)
        print('Vibrating')
    else:
        pwm.ChangeDutyCycle(0)
        print('Not Vibrating')