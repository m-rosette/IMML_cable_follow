import serial

# Open the serial port (replace 'COM3' with the appropriate port name on your system)
ser = serial.Serial('/dev/ttyACM1', 115200)  # Adjust baud rate if needed

while True:
    try:
        # Read a line from the serial port
        line = ser.readline().strip()  # Read bytes without decoding
        
        # Split the line into values using comma as delimiter
        values = line.split(b',')  # Use bytes literal for splitting
        
        # If two values are received (assuming button_value and currentPosition)
        if len(values) == 2:
            button_value, current_position = values
            # Now you can use button_value and current_position as needed
            print("Button value:", button_value.decode())
            print("Current position:", current_position.decode())
    except UnicodeDecodeError:
        print("Error decoding data from the serial port")

# Close the serial port when done
ser.close()
