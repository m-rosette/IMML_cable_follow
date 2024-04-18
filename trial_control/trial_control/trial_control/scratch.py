import numpy as np
import matplotlib.pyplot as plt

# Assuming you have loaded your data into numpy arrays 'timestamp' and 'cable_status'

# Generate some example data
timestamp = np.linspace(0, 10, 100)
cable_status = np.random.randint(0, 2, size=100)

# Initialize the plot
plt.figure(figsize=(10, 6))

# Plot the cable_status == 0 area with light grey
plt.fill_between(timestamp, 0, 1, where=(cable_status == 0), color='lightgrey', alpha=0.5, label='Cable Status 0')

# Plot the cable_status == 1 area with darker grey
plt.fill_between(timestamp, 0, 1, where=(cable_status == 1), color='darkgrey', alpha=0.5, label='Cable Status 1')

# Set labels and title
plt.xlabel('Timestamp')
plt.ylabel('Cable Status')
plt.title('Cable Status over Time')
plt.legend()

# Show the plot
plt.show()
