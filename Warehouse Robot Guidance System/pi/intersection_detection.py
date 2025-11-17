from TRSensors import TRSensor  # Import the TRSensor library
import time

# Initialize the TRSensors
tr_sensors = TRSensor()

# Define the threshold value for detecting an intersection
INTERSECTION_THRESHOLD = 950

def detect_intersection(sensor_values):
    """
    Detect if the robot is at an intersection by checking if all sensor values are below the threshold.
    """
    return all(value < INTERSECTION_THRESHOLD for value in sensor_values)

def main():
    print("Starting intersection detection test...")
    try:
        while True:
            # Read sensor values
            position, sensor_values = tr_sensors.readLine()
            
            # Print the sensor values for visual confirmation
            print(f"Sensor values: {sensor_values}")
            
            # Check for intersection
            if detect_intersection(sensor_values):
                print("Intersection detected!")
            else:
                print("No intersection detected.")
            
            # Wait for a bit before checking again
            time.sleep(1)

    except KeyboardInterrupt:
        print("Test interrupted.")

if __name__ == "__main__":
    main()
