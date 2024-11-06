from networktables import NetworkTables
import time

# Set up NetworkTables to connect to the localhost simulation
NetworkTables.initialize(server='127.0.0.1')  # localhost for simulation
table = NetworkTables.getTable('SmartDashboard')

# Wait until connected
while not NetworkTables.isConnected():
    print("Waiting for NetworkTables connection...")
    time.sleep(1)

print("Connected to NetworkTables!")

# Define the variable outside the loop
number = 1

# Publish values to SmartDashboard
while True:
    # Increment and publish the value
    table.putNumber('exampleNumber', number)
    print(f"Updated 'exampleNumber' to {number}")
    number += 1

    # Update other values
    table.putString('exampleString', 'Hello from Python!')

    # Update at intervals
    time.sleep(1)
