from vpython import canvas, box, cylinder, vector, rate

# Create the canvas
scene = canvas(title="3D Drone Visualization with Quadcopters",
               width=800, height=600, background=vector(0.8, 0.8, 0.8))
scene.camera.pos = vector(0, 5, 10)  # Set the camera starting position
scene.camera.axis = vector(0, -5, -10)  # Point the camera towards the origin

# Drone components (relative to a common origin)
drone_body = box(pos=vector(0, 0, 0), size=vector(2, 0.2, 2), color=vector(0, 0, 1))

arm1 = cylinder(pos=vector(-1, 0.1, 0), axis=vector(2, 0, 0), radius=0.05, color=vector(0.5, 0.5, 0.5))
arm2 = cylinder(pos=vector(0, 0.1, -1), axis=vector(0, 0, 2), radius=0.05, color=vector(0.5, 0.5, 0.5))

propeller1 = cylinder(pos=vector(-1.5, 0.15, 0), axis=vector(0, 0.01, 0), radius=0.3, color=vector(1, 0, 0))
propeller2 = cylinder(pos=vector(1.5, 0.15, 0), axis=vector(0, 0.01, 0), radius=0.3, color=vector(0, 1, 0))
propeller3 = cylinder(pos=vector(0, 0.15, -1.5), axis=vector(0, 0.01, 0), radius=0.3, color=vector(0, 0, 1))
propeller4 = cylinder(pos=vector(0, 0.15, 1.5), axis=vector(0, 0.01, 0), radius=0.3, color=vector(1, 1, 0))

# Group the drone components for consistent rotation
drone_components = [drone_body, arm1, arm2, propeller1, propeller2, propeller3, propeller4]

# Rotation variables
pitch = 0  # Rotation around x-axis
roll = 0   # Rotation around z-axis
yaw = 0    # Rotation around y-axis

# Track key states
keys_pressed = set()

# Key press event handler
def keydown(evt):
    global keys_pressed
    keys_pressed.add(evt.key)

# Key release event handler
def keyup(evt):
    global keys_pressed
    if evt.key in keys_pressed:
        keys_pressed.remove(evt.key)

# Bind the keydown and keyup events
scene.bind('keydown', keydown)
scene.bind('keyup', keyup)

# Animation loop
while True:
    rate(60)  # Limit to 60 frames per second

    # Update rotation based on keys pressed
    if 'w' in keys_pressed:  # Rotate pitch up
        pitch += 0.05
    if 's' in keys_pressed:  # Rotate pitch down
        pitch -= 0.05
    if 'a' in keys_pressed:  # Rotate yaw left
        yaw -= 0.05
    if 'd' in keys_pressed:  # Rotate yaw right
        yaw += 0.05
    if 'q' in keys_pressed:  # Rotate roll left
        roll -= 0.05
    if 'e' in keys_pressed:  # Rotate roll right
        roll += 0.05

    # Apply rotation to all drone components
    for component in drone_components:
        # Apply pitch rotation (x-axis)
        component.pos = component.pos.rotate(angle=pitch, axis=vector(1, 0, 0), origin=vector(0, 0, 0))
        # Apply yaw rotation (y-axis)
        component.pos = component.pos.rotate(angle=yaw, axis=vector(0, 1, 0), origin=vector(0, 0, 0))
        # Apply roll rotation (z-axis)
        component.pos = component.pos.rotate(angle=roll, axis=vector(0, 0, 1), origin=vector(0, 0, 0))

    # Rotate the propellers (independent spinning effect)
    propeller1.rotate(angle=0.3, axis=vector(0, 1, 0), origin=propeller1.pos)
    propeller2.rotate(angle=0.3, axis=vector(0, 1, 0), origin=propeller2.pos)
    propeller3.rotate(angle=0.3, axis=vector(0, 1, 0), origin=propeller3.pos)
    propeller4.rotate(angle=0.3, axis=vector(0, 1, 0), origin=propeller4.pos)