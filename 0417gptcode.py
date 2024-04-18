import cv2
from cvzone.FaceDetectionModule import FaceDetector
import serial
import numpy as np
import time
import math
import struct

# Setup serial connection
arduino_port = "/dev/cu.usbmodem1401"  # Update with your Arduino port
ser = serial.Serial(arduino_port, 9600, timeout=0.5)
time.sleep(5)

cap = cv2.VideoCapture(0)

ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

detector = FaceDetector()
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')  # Eye detector"""
servoPos = [90, 90, 90 , 90, 90] # initial servo position



# Known width of the face in real life (in centimeters)
KNOWN_WIDTH = 16.0  # You need to adjust this based on your actual conditions
# Initial distance from camera to face (you need to measure this)
KNOWN_DISTANCE = 60.0  # in centimeters
# Calculate the focal length (you should do this measurement once initially)
KNOWN_FACE_WIDTH = 300  # Known face width in pixels at 60 cm

focal_length = (KNOWN_FACE_WIDTH * KNOWN_DISTANCE) / KNOWN_WIDTH
initialized = False

# Variables for printing
variables = {
    "B": None, "servoX": None, "Lh": None, "A": None, "leftx": None,
    "Rh": None, "rightx": None, "lMotor": None, "rMotor":None, "dist": None, "servoY": None, "Vh": None,
    "Fh": None, "Fm": None, "firsty": None, "Sh": None, "Sm": None,
    "secy": None, "Th": None, "Tm": None, "thirdy":None,
}
while True:
    success, img = cap.read()
    img, bboxs = detector.findFaces(img, draw=False)
    eyes_detected = False  # Flag to indicate if eyes are detected

    if bboxs:
        for bbox in bboxs:
            x, y, w, h = bbox['bbox']
            face_roi = img[y:y+h, x:x+w]
            eyes = eye_cascade.detectMultiScale(face_roi, scaleFactor=1.1, minNeighbors=25)
            
            if len(eyes) > 0:  # Check if eyes are detected within the face
                eyes_detected = True
                break  # Exit the loop if eyes are detected

        if not eyes_detected:
            # Proceed with the face tracking logic if no eyes are detected
            fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
            pos = [fx, fy]

            fx = max(0, min(fx, ws))  # Ensure fx is within the range [0, ws]
            if fx == 0:
                fx = 1  # If fx is zero, set it to 1
            
            servoX = np.interp(fx, [1, ws], [180, 0])
            servoY = np.interp(fy, [1, hs], [0, 180])

            if servoX < 0:
                servoX = 1
            elif servoX > 180:
                servoX = 180
            if servoY < 0:
                servoY = 1
            elif servoY > 180:
                servoY = 180


            # Calculate depth based on the width of the face detected
            distance = (KNOWN_WIDTH * focal_length) / w
            cv2.putText(img, f"Depth: {distance:.2f} cm", (fx + 100, fy - 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 2)

            # Implementing new variables
            B = 50
            Lh = distance / (math.sin(math.radians(servoX)))
            A = math.sqrt(B**2 + (Lh**2) - 2*B*Lh*math.cos(math.radians(servoX)))
            leftx = math.degrees(math.asin(Lh * math.sin(math.radians(servoX)) / A))
            Rh = math.sqrt((2*B)**2 + A**2 - 2*(2*B)*A*math.cos(math.radians(leftx)))
            rightx = math.degrees(math.asin(A*math.sin(math.radians(leftx)) / Rh))

            rMotor = leftx
            lMotor = 180 - rightx

            dist = 30
            Vh = dist / (math.sin(math.radians(servoY)))
            Fh = math.sqrt(Vh**2 + dist**2 - 2*Vh*dist*math.cos(math.radians(servoY)))
            Fm = math.degrees(math.asin(Vh*math.sin(math.radians(servoY)) / Fh))
            firsty = 180 - Fm
            Sh = math.sqrt(Fh**2 + dist**2 - 2*Fh*dist*math.cos(math.radians(firsty)))
            Sm = math.degrees(math.asin(Fh*math.sin(math.radians(firsty)) / Sh))
            secy = 180 - Sm
            Th = math.sqrt(Sh**2 + dist**2 - 2*Sh*dist*math.cos(math.radians(secy)))
            Tm = math.degrees(math.asin(Sh*math.sin(math.radians(secy)) / Th))
            thirdy = 180 - Tm

            """if rMotor < 0:
                rMotor = 0
            elif rMotor > 180:
                rMotor = 180
            if lMotor < 0:
                lMotor = 0
            elif lMotor > 180:
                lMotor = 180
            if firsty < 0:
                firsty = 0
            elif firsty > 180:
                firsty = 180
            if secy < 0:
                secy = 0
            elif secy > 180:
                secy = 180
            if thirdy < 0:
                thirdy = 0
            elif thirdy > 180:
                thirdy = 180
           
            servoPos[0] = rMotor
            servoPos[1] = lMotor
            servoPos[2] = firsty
            servoPos[3] = secy
            servoPos[4] = thirdy
            """

            servoPos = [rMotor, lMotor, firsty, secy, thirdy]
            rMotorInt = round(rMotor)
            lMotorInt = round(lMotor)
            firstyInt = round(firsty)
            secyInt = round(secy)
            thirdyInt = round(thirdy)
            
            
            # Update variables dictionary
            variables.update({
                "B": B, "servoX": servoX, "Lh": Lh, "A": A, "leftx": leftx,
                "Rh": Rh, "rightx": rightx, "lMotor": lMotor, "rMotor": rMotor, "dist": dist, "servoY": servoY, "Vh": Vh,
                "Fh": Fh, "Fm": Fm, "firsty": firsty, "Sh": Sh, "Sm": Sm,
                "secy": secy, "Th": Th, "Tm": Tm, "thirdy": thirdy,
            })

            # Print variables
            '''for var, value in variables.items():
                print(f"{var}: {value}")'''

            cv2.circle(img, (int(leftx), int(servoY)), 5, (0, 255, 0), -1)
            cv2.circle(img, (int(rightx), int(servoY)), 5, (0, 255, 0), -1)
            cv2.circle(img, (int(servoX), int(firsty)), 5, (0, 255, 0), -1)
            cv2.circle(img, (int(servoX), int(secy)), 5, (0, 255, 0), -1)
            

            cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

            if not initialized:
                """ser.write(f"{int(servoPos[0])},{int(servoPos[1])},{int(servoPos[2])},{int(servoPos[3])},{int(servoPos[4])}\n".encode())"""
                ser.write(struct.pack('<5i', rMotorInt, lMotorInt, firstyInt, secyInt, thirdyInt))
                print(f"{int(servoPos[0])},{int(servoPos[1])},{int(servoPos[2])},{int(servoPos[3])},{int(servoPos[4])}")
        
                initialized = True
            else:
                initialized = False


        else:
            # Add a visual/text cue or logic for when eyes are detected but face tracking is not performed
            cv2.putText(img, "EYES DETECTED - TRACKING PAUSED", (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
    else:
        cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)


    cv2.imshow("Image", img)
    cv2.waitKey(1)
    

cap.release()
cv2.destroyAllWindows()
