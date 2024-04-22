# import serial

# # Open the serial port (replace 'COM3' with the appropriate port name on your system)
# ser = serial.Serial('/dev/ttyACM1', 115200)  # Adjust baud rate if needed

# while True:
#     try:
#         # Read a line from the serial port
#         line = ser.readline().strip()  # Read bytes without decoding
        
#         # Split the line into values using comma as delimiter
#         values = line.split(b',')  # Use bytes literal for splitting
        
#         # If two values are received (assuming button_value and currentPosition)
#         if len(values) == 2:
#             button_value, current_position = values
#             # Now you can use button_value and current_position as needed
#             print("Button value:", button_value.decode())
#             print("Current position:", current_position.decode())
#     except UnicodeDecodeError:
#         print("Error decoding data from the serial port")

# # Close the serial port when done
# ser.close()

def calculate_slope(list1, list2):
    # Check if the lists have the same length
    if len(list1) != len(list2):
        raise ValueError("Lists must have the same length")
    
    slopes = []
    for i in range(1, len(list1)):
        numerator = list1[i] - list1[i-1]
        denominator = list2[i] - list2[i-1]
        
        # Avoid division by zero
        if denominator == 0:
            raise ValueError("Denominator cannot be zero")
        
        slope = numerator / denominator
        slopes.append(slope)
    
    return slopes

# Example usage:
list1 = [1, 3, 5, 7, 9]
list2 = [2, 4, 6, 8, 10]
slopes = calculate_slope(list1, list2)
print("Slopes:", slopes)
print(len(list1))
