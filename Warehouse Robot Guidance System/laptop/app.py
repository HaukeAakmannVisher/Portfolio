import RPi.GPIO as GPIO
import time
import requests
from flask import Flask, request, jsonify
import threading

# Set up ultrasonic sensor pins
TRIG = 22
ECHO = 27
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN)

# Initialize the TRSensors object for intersection detection
from TRSensors import TRSensor
tr_sensors = TRSensor()

# Initialize Flask app
app = Flask(__name__)

tasks = []  # Global task list
current_task_index = 0
intersection_detected = False  # Track intersection detection
obstacle_detected = False  # Track obstacle detection

# Function to measure distance using the ultrasonic sensor
def dist():
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TRIG, GPIO.LOW)

    while not GPIO.input(ECHO):
        pass
    t1 = time.time()

    while GPIO.input(ECHO):
        pass
    t2 = time.time()

    return (t2 - t1) * 34000 / 2
    
def clear_task_list():
    global tasks, current_task_index
    tasks = []  # Clear the tasks
    current_task_index = 0  # Reset the task index
    obstacle_detected = False # once the list is cleared again
    print("Task list cleared after obstacle detection.")

    
# Function to check for obstacles (Sends obstacle detection log message)
def check_for_obstacles():
    global obstacle_detected
    while current_task_index < len(tasks):  # Loop to continuously check for obstacles until tasks are done
        distance = dist()
        # If an obstacle is detected, log and stop further actions
        if distance < 5 and not obstacle_detected:  # Example threshold for obstacle detection
            obstacle_detected = True
            task_position = tasks[current_task_index].split(' at ')[-1]  # Extract current task position

            # Handle cases where the task is "stop" (no valid position)
            if "stop" in task_position:
                send_update_to_laptop(current_task_index + 1, "N/A", "Obstacle detected", obstacle_detected=True)
            else:
                try:
                    position = eval(task_position)  # Safely evaluate the position if it's not "stop"
                    send_update_to_laptop(current_task_index + 1, position, "Obstacle detected", obstacle_detected=True)
                except Exception as e:
                    print(f"Error processing task position: {e}")

            print(f"Obstacle detected before Position {task_position}!")
        elif distance >= 5 and obstacle_detected:
            # Clear obstacle once distance is safe
            obstacle_detected = False
            print("Obstacle cleared.")
        time.sleep(0.5)  # Delay to prevent overwhelming the sensor




# Function to detect intersections (Now sends intersection log message with action details)
def detect_intersection(sensor_values):
    global intersection_detected

    # Intersection is detected if 3 or more sensor values are less than a threshold
    below_threshold = [value for value in sensor_values if value < 700]  # Modify threshold as needed

    if len(below_threshold) >= 3 and not intersection_detected:
        intersection_detected = True

        # Extract position and action from the current task
        task = tasks[current_task_index]  # Get the current task

        if "stop" in task:
            position = "N/A"
            action = "stop"
        else:
            try:
                action, position = task.split(' at ')  # Split the task to extract the action and position
                position = eval(position)  # Safely convert position string to list
            except Exception as e:
                generate_logs(f"Error processing intersection task position: {e}")
                position = [0, 0]  # Fallback to a default if parsing fails
                action = "unknown"

        # Create a single message that includes intersection detection
        send_update_to_laptop(current_task_index + 1, position, action, intersection_detected=True)
        return True
    else:
        intersection_detected = False
    return False

# Function to execute tasks
def execute_task():
    global current_task_index
    if current_task_index < len(tasks):
        if obstacle_detected:
            print("Obstacle detected, halting task execution")  # Debugging info
            return  # Halt task execution if an obstacle is detected

        task = tasks[current_task_index]

        if "stop" in task:
            generate_logs(f"Executing stop at task {current_task_index + 1}")
            #send_update_to_laptop(current_task_index + 1, "N/A", "stop")
        else:
            try:
                action, position = task.split(' at ')
                position = eval(position)  # Safely convert position string to list
               # send_update_to_laptop(current_task_index + 1, position, action)
            except Exception as e:
                generate_logs(f"Error executing task: {e}")

        current_task_index += 1

# Continuously check for intersections and execute tasks
def wait_for_intersection():
    global current_task_index, tasks
    while current_task_index < len(tasks):
        # Read sensor values from the TR sensors to detect the intersection
        position, sensor_values = tr_sensors.readLine()

        # Debug: print sensor values for observation
        # print(f"Sensor values: {sensor_values}")

        # Check for intersections while waiting, but only proceed if no obstacles
        if not obstacle_detected and detect_intersection(sensor_values):
            # Execute the next task if an intersection is detected
            execute_task()
            time.sleep(2)  # Short delay after executing the task

        time.sleep(0.5)  # Slow down the loop slightly to avoid overwhelming the CPU


# Function to generate logs
def generate_logs(message):
    print(f"{message} at {time.strftime('%H:%M:%S')}")  # Print to console

# Function to send updates to the laptop (handles both intersection and task updates)
def send_update_to_laptop(task_num, position, action, obstacle_detected=False, intersection_detected=False):
    try:
        laptop_url = "http://192.168.137.1:5000/update_log"  # Replace with actual laptop IP

        # Ensure a message is created based on the conditions
        if obstacle_detected:
            message = f"Obstacle detected before ({position[0]}, {position[1]})"
        elif intersection_detected:
            # Send a single intersection message, including position and action
            message = f"Intersection detected: Position ({position[0]}, {position[1]}), {action}"
        else:
            # Default case for regular instruction updates
            message = f"Instruction {task_num}, Position ({position[0]}, {position[1]}), {action}"

        # Send the message to the laptop
        payload = {"message": message}
        response = requests.post(laptop_url, json=payload)

        if response.status_code == 200:
            generate_logs(f"Successfully sent update to laptop: {message}")
        else:
            generate_logs(f"Failed to send update to laptop. Status code: {response.status_code}")
    except Exception as e:
        generate_logs(f"Error sending update to laptop: {e}")




@app.route('/receive_tasks', methods=["POST"])
def receive_tasks():
    global tasks, current_task_index
    try:
        task_data = request.get_json()

        # Log that the task list has been received
        generate_logs(f"Received task data from laptop: {task_data}")
        print("Task list received!")

        # Extract tasks from the received data
        tasks1 = task_data.get("tasks1", [])
        tasks2 = task_data.get("tasks2", [])
        tasks3 = task_data.get("tasks3", [])
        tasks = tasks1 + tasks2 + tasks3  # Combine all tasks into a single list
        current_task_index = 0  # Reset task index

        generate_logs(f"Task list: {tasks}")

        # If a new task list is received, start both obstacle and intersection detection concurrently
        obstacle_thread = threading.Thread(target=check_for_obstacles, daemon=True)
        obstacle_thread.start()

        wait_for_intersection()

        return jsonify({"status": "Task list received and waiting for intersections"}), 200
    except Exception as e:
        generate_logs(f"Error receiving tasks: {e}")
        print(f"Error receiving tasks: {e}")  # Debug: Print error details
        return jsonify({"error": "Failed to receive task list"}), 500
if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=True)
