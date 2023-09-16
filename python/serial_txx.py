import serial

# Define the serial port and baud rate (replace 'COMx' with the correct COM port)
serial_port = 'COM3'
baud_rate = 1000000

# Initialize the serial connection
ser = serial.Serial(serial_port, baud_rate)
variable = []
try:
    while True:
        # Read data from the serial port
        data = ser.readline().decode().strip()
        
        # Process and print the received data (you can modify this part)
        variable.append(data)
        print("Received Data:", data)

# i need to graph the data in variable "data"
        
except KeyboardInterrupt:
    # Close the serial port when the script is interrupted (e.g., by pressing Ctrl+C)
    arch = open("salida.csv",'w')
    for a in variable:
        arch.write(a+'\n')
    ser.close()