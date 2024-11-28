import PySimpleGUI as sg
import firebase_admin
from firebase_admin import credentials, db
import math
from time import sleep

# Firebase setup
cred = credentials.Certificate('key.json')

firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://projectiot-ee562-default-rtdb.europe-west1.firebasedatabase.app/'
})
ref = db.reference('/SensorData')

# PySimpleGUI setup
layout = [
    [sg.Text("Drone Orientation Visualization", size=(30, 1), justification="center", font="Any 20")],
    [sg.Canvas(size=(800, 600), background_color="black", key="-CANVAS-")],
    [sg.Button("Exit", size=(10, 1))]
]

window = sg.Window("Drone Visualization", layout, finalize=True, resizable=True)
canvas = window["-CANVAS-"].TKCanvas

# Function to fetch yaw, pitch, roll from Firebase
def fetch_orientation():
    try:
        data = ref.get()  # Get data from Firebase
        if data:
            yaw = data.get('Yaw', 0)
            pitch = data.get('Pitch', 0)
            roll = data.get('Roll', 0)
            print(f"Yaw: {yaw}, Pitch: {pitch}, Roll: {roll}")  # Debug output
            return yaw, pitch, roll
        print("No data found in Firebase.")  # Debug output
    except Exception as e:
        print(f"Error fetching data: {e}")
    return 0, 0, 0

# Function to rotate a point around the center
def rotate_point(x, y, angle, cx, cy):
    angle_rad = math.radians(angle)
    x -= cx
    y -= cy
    new_x = x * math.cos(angle_rad) - y * math.sin(angle_rad)
    new_y = x * math.sin(angle_rad) + y * math.cos(angle_rad)
    return new_x + cx, new_y + cy

# Function to draw the drone on the canvas
def draw_drone(canvas, yaw, pitch, roll):
    canvas.delete("all")  # Clear canvas
    
    # Define drone body and arms
    center_x, center_y = 400, 300
    body_width, body_height = 60, 20
    arm_length = 80

    # Calculate corners of the body rectangle
    body_coords = [
        (center_x - body_width // 2, center_y - body_height // 2),
        (center_x + body_width // 2, center_y - body_height // 2),
        (center_x + body_width // 2, center_y + body_height // 2),
        (center_x - body_width // 2, center_y + body_height // 2),
    ]
    
    # Rotate body corners based on yaw
    rotated_body = [rotate_point(x, y, yaw, center_x, center_y) for x, y in body_coords]
    
    # Draw drone body
    canvas.create_polygon(
        *[(x, y) for x, y in rotated_body],
        fill="green"
    )
    
    # Draw arms based on pitch and roll
    arm_coords = [
        (center_x - arm_length, center_y),
        (center_x + arm_length, center_y),
        (center_x, center_y - arm_length),
        (center_x, center_y + arm_length),
    ]
    
    rotated_arms = [
        rotate_point(x, y, yaw, center_x, center_y) for x, y in arm_coords
    ]
    
    # Draw horizontal arm
    canvas.create_line(
        rotated_arms[0][0], rotated_arms[0][1], rotated_arms[1][0], rotated_arms[1][1],
        fill="red", width=2
    )
    # Draw vertical arm
    canvas.create_line(
        rotated_arms[2][0], rotated_arms[2][1], rotated_arms[3][0], rotated_arms[3][1],
        fill="blue", width=2
    )

# Main loop
try:
    while True:
        event, _ = window.read(timeout=50)  # Event loop with a 50ms timeout
        
        if event == sg.WINDOW_CLOSED or event == "Exit":
            break
        
        # Fetch orientation from Firebase
        yaw, pitch, roll = fetch_orientation()

        # Draw the drone on the canvas
        draw_drone(canvas, yaw, pitch, roll)
        
        sleep(0.1)  # Slight delay to avoid excessive Firebase calls

finally:
    window.close()
