import numpy as np

# Calculations are made based on `ROBOTIQ_AGS-001-XXXX_20171116.STEP`
# Coordinates are in mm
# y-coordinate points up

def calculate_angle_and_dist(AGS_finger_prox, AGS_finger_dist, dist_dx):
    dx = dist_dx

    alpha = np.arctan2(AGS_finger_dist[1] - AGS_finger_prox[1], AGS_finger_dist[0] - AGS_finger_prox[0])

    rx = AGS_finger_dist[0] - AGS_finger_prox[0]
    ry = AGS_finger_dist[1] - AGS_finger_prox[1]
    R = np.sqrt(rx**2 + ry**2)

    theta = np.arccos((dx+R*np.cos(alpha)) / R) - alpha 

    dy = R * np.sin(theta + alpha) - R*np.sin(alpha)
    return theta, dy

dx = 12.5
AGS_finger_prox = (-44.53, 82.63, 36.51)
AGS_finger_dist = (-98.72, 160.03, 36.51)
theta, dy = calculate_angle_and_dist(AGS_finger_prox, AGS_finger_dist, dx)
print("Finger AGS_finger_prox:")
print(f"Angle (degrees): %.4f, Dy: %.4f" % (np.degrees(theta), dy))

dx = 12.5
AGS_finger_prox001 = (-44.53, 82.63, -36.51)
AGS_finger_dist001 = (-99.16, 160.66, -36.51)
theta, dy = calculate_angle_and_dist(AGS_finger_prox001, AGS_finger_dist001, dx)
print("\nFinger AGS_finger_prox001:")
print(f"Angle (degrees): %.4f, Dy: %.4f" % (np.degrees(theta), dy))

dx = -12.5
AGS_finger_prox002 = (44.48, 82.63, 0.0)
AGS_finger_dist002 = (98.67, 160.03, 0.0)
theta, dy = calculate_angle_and_dist(AGS_finger_prox002, AGS_finger_dist002, dx)
print("\nFinger AGS_finger_prox002:")
print(f"Angle (degrees): %.4f, Dy: %.4f" % (np.degrees(theta), dy))
