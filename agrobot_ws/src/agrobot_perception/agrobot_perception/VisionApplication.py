import math
import os
import time
import cv2
import numpy as np
import queue
import sqlite3

# this is a test to see if the edits work
class DataBase(object):
    
    def __init__(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        db_path = os.path.join(script_dir, "hsv.db")

        self.conn = sqlite3.connect(db_path)
        self.cursor = self.conn.cursor()
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS hsv (
                hsvColor TEXT PRIMARY KEY,
                hueMin INTEGER,
                satMin INTEGER,
                valMin INTEGER,
                hueMax INTEGER,
                satMax INTEGER,
                valMax INTEGER
            )
        ''')

    def updateColorList(self,newColorArray):
        # Takes a color and its hsv values and adds it to the database, unless the color already exists.
        # Ex array: ("Green",0,0,0,255,255,255)
        
        self.cursor.execute('''
            INSERT OR REPLACE INTO hsv 
            (hsvColor,hueMin,satMin,valMin,hueMax,satMax,valMax) 
            VALUES (?,?,?,?,?,?,?)
        ''',newColorArray)
        self.debugPrint()

    def readhsvValues(self):
        # Returns a list of all of the colors and the masking values from the database (2d list)
        self.cursor.execute(f"SELECT * FROM hsv")
        colors = self.cursor.fetchall()
        colorDict = {}
        for color in colors:
            colorDict[color[0]] = color[1:]
        return colorDict

    def commitAndClose(self):
        self.conn.commit()
        self.conn.close()

    def debugPrint(self):
        self.cursor.execute("SELECT * FROM hsv")
        results = self.cursor.fetchall()
        for row in results:
            print(row)


class CameraView(object):
    def __init__(self, camera, vertFOV, horizFOV, elevationOfTarget, elevationOfCamera, angleFromHoriz):
        self.cap = camera
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.vertFOV = vertFOV
        self.horizFOV = horizFOV
        self.elevationOfTarget = elevationOfTarget
        self.elevationOfCamera = elevationOfCamera
        self.angleFromHoriz = angleFromHoriz
        self.cameraCenter = self.width/2
        self.radiusFromAxisOfRotation = 14/12 # measured in feet (distance from camera to the axis of rotation of the robot)

# A class used to describe an object detected by its color (hsv) values.
class TapeTarget(object):
    def __init__(self, imageResult, approx, tapeTargetDetected, camera, areaR, targetC):
        self.tapeTargetDetected = tapeTargetDetected
        self.imageResult = imageResult
        if self.tapeTargetDetected:
            self.x, self.y, self.w, self.h, = cv2.boundingRect(approx)
        else:
            self.x, self.y, self.w, self.h, = 1, 1, 1, 1 
        self.boundingArea = self.w * self.h
        self.normalizedY = (self.y - camera.height/2)/(camera.height/2) * -1
        self.normalizedX = (self.x - camera.width/2)/(camera.width/2)
        self.pitch = (self.normalizedY/2) * camera.vertFOV
        self.yaw = (self.normalizedX/2) * camera.horizFOV
        self.offset = self.x + self.w/2 - camera.cameraCenter
        self.aspectRatio = self.w/self.h
        self.areaRatio = areaR
        self.boundingArea = self.w * self.h
        #(height of target [feet] - height of camera [feet])/tan(pitch [degrees] + angle of camera [degrees])
        self.distanceToTarget = (camera.elevationOfTarget - camera.elevationOfCamera) / math.tan(math.radians(self.pitch + camera.angleFromHoriz))
        self.targetColor = targetC


    def drawRectangle(self):
        # Draw rectangle on the Image
        cv2.rectangle(self.imageResult, (self.x,self.y),(self.x+self.w,self.y+self.h),(0,255,0),3)
        cv2.putText(self.imageResult,self.targetColor,(self.x+20,self.y+20),cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,0),2)


class VisionApplication(object):
    def __init__(self):
        self.imgResult = None

        self.tapeTargetDetected = False

        self.distanceFromTarget = 0

        self.processingForColor = True

        # Initialize configuration

        self.imgResult = None
        #self.mask = None

        self.cameraInUse = 1

        # Set Number of Cameras
        ##### ****************** ##
        self.numberOfCameras = 1 ##
        ##### ****************** ##

        self.detectionMode = 0
       
        
        self.masksArray = []
        self.hsvDataBase = DataBase()
        self.myColors = self.hsvDataBase.readhsvValues()

            

        self.areaRatio = 0 # this is the areaRatio of every contour that is seen by the camera
        self.largestAreaRatio = 0 # this is the areaRatio of the target once it has been isolated
        self.aspectRatio = 0 # this is the aspectRatio of every contour that is seen by the camera (width/height)
        self.largestAspectRatio = 0 # this is the aspectRatio fo the target once it has been isolated

        # More reasonable detection constants for general object detection
        # [idealAreaRatio, areaTolerance, idealAspectRatio, aspectTolerance, idealY, yTolerance, idealX, xTolerance]
        self.colorDetectConstants = [.6, 4, .3, 4, 90, 200, -1, 200]
        self.garea = 500
        self.contours = None
        self.targets = {}
        self.tapeTargetList = []

        self.running = True

        self.hsv_queue = queue.Queue(maxsize=1)
        self.frame_queue = queue.Queue(maxsize=1)

        

        #TODO: Fill out values below if distance calculation is desired. The first value is for camera #1, the second is for camera #2. If no second camera exists, set all the second values to 1.
        # Distance Calculation Constants
        #Vertical Field of View (Degrees)
        vertFOV = [48.94175846, 1]

        #Horizontal Field of View (Degrees)
        horizFOV = [134.3449419, 1]

        #Height of the target off the ground (feet)
        elevationOfTarget = [1.5, 1]

        #Height of the Camera off the ground (feet)
        elevationOfCamera = [0.9, 1] 

        #Angle the camera makes relative to the horizontal (degrees)
        angleFromHoriz = [30, 1]


        self.camera = CameraView(cv2.VideoCapture(0), vertFOV[0], horizFOV[0], elevationOfTarget[0], elevationOfCamera[0], angleFromHoriz[0])
        if self.numberOfCameras == 2:
            self.camera2 = CameraView(cv2.VideoCapture(1), vertFOV[1], horizFOV[1], elevationOfTarget[1], elevationOfCamera[1], angleFromHoriz[1])

        self.currentCamera = self.camera


    def getImageMask(self, img, myColors):
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  
        masks = {}
        for colorName, color in myColors.items() :

            lower = np.array(color[0:3], dtype=np.uint8)
            upper = np.array(color[3:6], dtype=np.uint8)
            masks[colorName] = cv2.inRange(imgHSV, lower, upper) 
        
        return masks


    def getContours(self, maskDictionary):
        contourDict = {}
        #Takes the array of masked images and returns an array of countours based off of that.
        for maskName, mask in maskDictionary.items():

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            contourDict[maskName] = contours
        # print(f"Found {len(contours)} contours")
        return contourDict

    def isolateTarget(self, contourDict):
        
        idealAreaRatio = self.colorDetectConstants[0] # this is the ideal ratio for the area ratio value
        areaTolerance = self.colorDetectConstants[1] # this is the tolerance for finding the target with the right aspect ratio
        
        idealAspectRatio = self.colorDetectConstants[2] # this is the ideal aspect ratio based off of the diagram but can be changed as needed.
        aspectTolerance = self.colorDetectConstants[3]

        

        # idealYCoor is the Y coordinate where the target should usually be
        if self.colorDetectConstants[4] == -1:
            idealYCoor = self.currentCamera.height/2
        else:
            idealYCoor = self.colorDetectConstants[4]
        # yCoorTolerance is added and subtracted from the ideal coordinate to create a range on the y axis where the target should be.
        # Any target detected outside that range is ignored
        yCoorTolerance = self.colorDetectConstants[5]

        # idealXCoor is the X coordinate where the target should usually be
        if self.colorDetectConstants[6] == -1:
            idealXCoor = self.currentCamera.width/2
        else:
            idealXCoor = self.colorDetectConstants[6]
        # xCoorTolerance is added and subtracted from the ideal coordinate to create a range on the x axis where the target should be.
        # Any target detected outside that range is ignored
        xCoorTolerance = self.colorDetectConstants[7]

        # start off with a large tolerance, and if the ideal ratio is correct, lower the tolerance as needed. 
        self.targets = {}
        
        self.tapeTargetDetected = False
        for colorName, contours in contourDict.items():
            
            if contours is not None and len(contours) > 0:
                largest = contours[0]
                area = 0
                
                for contour in contours:
                    try:
                        # Validate contour before processing
                        if contour is None or len(contour) < 3:
                            continue
                        
                        # cv2.findContours returns contours in a specific format
                        # We need to ensure the contour is properly formatted for cv2.contourArea
                        # The issue is that cv2.contourArea expects contours to be in CV_32F or CV_32S format
                        # Let's try a different approach - use the contour as-is but ensure it's valid
                        if contour.dtype != np.float32 and contour.dtype != np.int32:
                            # Convert to the format that cv2.contourArea expects
                            contour = contour.astype(np.float32)
                        
                        # Check if contour has valid points
                        if contour.shape[0] < 3:
                            continue
                        
                        contourArea = cv2.contourArea(contour) #area of the particle
                        if contourArea <= 0:
                            continue
                            
                        x, y, w, h, = cv2.boundingRect(contour)
                        if w <= 0 or h <= 0:
                            continue
                            
                    except Exception as e:
                        print(f"Invalid contour: {e}")
                        continue
                    boundingArea = w * h
                    # ignores targets that are too small
                    if (boundingArea < self.garea):
                        continue
                    #ignores targets that are outside a predetermined range

                    self.areaRatio = contourArea/boundingArea
                    self.aspectRatio = w/h
                    
                    # Print all contours with their area and aspect ratios
                    #print(f"Contour: AreaRatio={self.areaRatio:.3f}, AspectRatio={self.aspectRatio:.3f}, Area={contourArea:.0f}, BoundingArea={boundingArea:.0f}, W={w}, H={h}")
                    if self.areaRatio > idealAreaRatio - areaTolerance and self.areaRatio < idealAreaRatio + areaTolerance: # if the targets are within the right area ratio range, they is possibly the correct target
                        if self.aspectRatio > idealAspectRatio - aspectTolerance and self.aspectRatio < idealAspectRatio + aspectTolerance: # if the target is within the correct aspect ratio range aswell, it is definitely the right target
                            #print(f"*** TARGET DETECTED! *** AreaRatio={self.areaRatio:.3f}, AspectRatio={self.aspectRatio:.3f}")
                            largest = contour
                            self.tapeTargetDetected = True
                            
                            self.targets[colorName] = contour
                            # Draw the contours
                            cv2.drawContours(self.imgResult, largest, -1, (255,0,0), 3)

    def drawBoundingBox(self):
        if self.tapeTargetDetected:
            for targetColor, target in self.targets.items():
                try:
                    peri = cv2.arcLength(target, True)
                except:
                    print("CV2 Error")
                    continue
                approx = cv2.approxPolyDP(target, 0.02 * peri, True)
                x, y, w, h, = cv2.boundingRect(target)
                boundingArea = w * h
                contourArea = cv2.contourArea(target)
                self.tapeTargetList.append(TapeTarget(self.imgResult, approx, self.tapeTargetDetected, self.camera,(contourArea/boundingArea),targetColor))
        else:
            approx = None

    def processImgForColor(self, input_img):
        
        self.masksArray = self.getImageMask(input_img,self.myColors)
        self.contours = self.getContours(self.masksArray)
        self.isolateTarget(self.contours)
        self.drawBoundingBox()


    def cameraThread(self):
        input_img1 = np.zeros(shape=(self.camera.height,self.camera.width,3),dtype=np.uint8)
        targetDetTol = 1.0 

        print("Camera thread started")
        while self.running:
            # Get HSV values from GUI (non-blocking)

            # If frame is read correctly, ret is True
            
            rescalePercentage = .3
            if self.cameraInUse == 1 or self.numberOfCameras == 1:
                #this is where the actual video is passed in.
                ret, input_img1 = self.camera.cap.read()
                if not ret or input_img1 is None:
                    print("Failed to read from camera 1")
                    time.sleep(0.1)
                    continue
                input_img1 = cv2.resize(input_img1, (int(self.camera.width*rescalePercentage),int(self.camera.height*rescalePercentage)), interpolation = cv2.INTER_AREA)
            else:
                ret, input_img1 = self.camera2.cap.read()
                if not ret or input_img1 is None:
                    print("Failed to read from camera 2")
                    time.sleep(0.1)
                    continue
                input_img1 = cv2.resize(input_img1, (self.camera2.width,self.camera2.height), interpolation = cv2.INTER_AREA)
            
            self.imgResult = input_img1.copy()

            if self.processingForColor:
                self.tapeTargetList = []
                self.targets = []
                self.processImgForColor(input_img1)
                # sorts the list of tape targets from left to right
                
                if self.tapeTargetDetected:
                    """
                    self.tapeTargetList.sort(key=lambda target: target.boundingArea) # Sorts the targets smallest to largest.
                    targetID = len(self.tapeTargetList)-1
                    self.tapeTargetList[targetID].drawRectangle()
                    """
                    for target in self.tapeTargetList:
                        target.drawRectangle()
                        print(f"{target.targetColor}: {target.x},{target.y}")
                    # Data published for Color    
                # Scale by factor (0.5 = half size, 2.0 = double size)
                scale_factor = 1
                counter = 0
                for maskName, mask in self.masksArray.items():
                    counter += 1
                    new_width = int(mask.shape[1] * scale_factor)
                    new_height = int(mask.shape[0] * scale_factor)

                    resizedMask = cv2.resize(mask, (new_width, new_height))
                    #cv2.imshow(f'Mask Video + {maskName}', resizedMask)
                
                # Resize result image
                new_width = int(self.imgResult.shape[1] * scale_factor)
                new_height = int(self.imgResult.shape[0] * scale_factor)
                resizedResult = cv2.resize(self.imgResult, (new_width, new_height))

                
                #cv2.imshow('Result Video', resizedResult)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Quit key pressed")
                    self.running = False
                    break

            
            
            # Small delay to prevent overwhelming the system
            time.sleep(0.1)  # ~30 FPS


    def runApplication(self):
        print("Starting application...")

        try:
            self.cameraThread()
        except KeyboardInterrupt:
            print("Application interrupted by user")
        finally:
            self.hsvDataBase.commitAndClose()
            # Cleanup
            print("Cleaning up...")
            self.running = False
            if hasattr(self, 'camera') and self.camera:
                self.camera.cap.release()
            cv2.destroyAllWindows()
            print("Application ended")

def main():
    visionApp = VisionApplication()

    visionApp.runApplication()

main()   


