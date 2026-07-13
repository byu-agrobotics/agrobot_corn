"""Vision module adapted from agrobot_perception VisionApplication."""

import math
import os
import time
import cv2
import numpy as np
import sqlite3
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

try:
    from ament_index_python.packages import get_package_share_directory
    HAS_AMENT = True
except ImportError:
    HAS_AMENT = False

try:
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
except ImportError:
    HAS_PICAMERA2 = False


def _get_vision_data_dir():
    """Get vision data directory. Tries package share/data first, then package root data."""
    if HAS_AMENT:
        try:
            pkg_share = get_package_share_directory('agrobot_detect')
            data_dir = os.path.join(pkg_share, 'data')
            if os.path.isdir(data_dir) and os.path.exists(os.path.join(data_dir, 'hsv.db')):
                return data_dir
        except Exception:
            pass
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Package root data (agrobot_detect/data/hsv.db when script is in agrobot_detect/agrobot_detect/)
    parent_data = os.path.join(os.path.dirname(script_dir), 'data')
    if os.path.exists(os.path.join(parent_data, 'hsv.db')):
        return parent_data
    # Fallback: script_dir/data then script_dir
    data_subdir = os.path.join(script_dir, 'data')
    if os.path.exists(os.path.join(data_subdir, 'hsv.db')):
        return data_subdir
    return script_dir


class PiCameraCapture:
    """OpenCV-like capture wrapper for Picamera2."""

    def __init__(self, camera_index=0, width=640, height=480):
        self.width = width
        self.height = height
        self.camera_index = camera_index
        self.picam2 = Picamera2(camera_num=camera_index)
        config = self.picam2.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()

    def read(self):
        frame = self.picam2.capture_array()
        if frame is None:
            return False, None
        # picamera2 returns RGB, convert to BGR for OpenCV processing.
        return True, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    def get(self, prop):
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self.width)
        if prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self.height)
        return 0.0

    def release(self):
        try:
            self.picam2.stop()
        except Exception:
            pass


def create_camera_capture(use_pi_camera=True, camera_index=0, width=640, height=480):
    if use_pi_camera:
        if not HAS_PICAMERA2:
            raise RuntimeError("picamera2 is not installed. Run with --usb or install python3-picamera2.")
        return PiCameraCapture(camera_index=camera_index, width=width, height=height)

    # Use V4L2 backend for Pi camera compatibility in Docker (avoids GStreamer issues)
    device = camera_index if isinstance(camera_index, int) else camera_index
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


class StreamingHandler(BaseHTTPRequestHandler):
    # Class variable to store the latest frame (shared across all handlers)
    latest_frame = None
    frame_lock = threading.Lock()

    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()

            try:
                while True:
                    # Get the latest frame (thread-safe)
                    with StreamingHandler.frame_lock:
                        frame = StreamingHandler.latest_frame

                    if frame is not None:
                        # Encode frame as JPEG
                        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        frame_bytes = buffer.tobytes()

                        # Send frame
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', len(frame_bytes))
                        self.end_headers()
                        self.wfile.write(frame_bytes)
                        self.wfile.write(b'\r\n')

                    time.sleep(0.033)  # ~30 FPS
            except (ConnectionResetError, BrokenPipeError):
                # Client disconnected
                pass
        else:
            self.send_response(404)
            self.end_headers()


class DataBase(object):

    def __init__(self, dbName):
        data_dir = _get_vision_data_dir()
        db_path = os.path.join(data_dir, f"{dbName}.db")

        self.conn = sqlite3.connect(db_path)
        self.cursor = self.conn.cursor()
        self.cursor.execute(f'''
            CREATE TABLE IF NOT EXISTS {dbName} (
                hsvColor TEXT PRIMARY KEY,
                hueMin INTEGER,
                satMin INTEGER,
                valMin INTEGER,
                hueMax INTEGER,
                satMax INTEGER,
                valMax INTEGER
            )
        ''')

    def updateColorList(self, newColorArray):
        # Takes a color and its hsv values and adds it to the database, unless the color already exists.
        # Ex array: ("Green",0,0,0,255,255,255)

        self.cursor.execute('''
            INSERT OR REPLACE INTO hsv
            (hsvColor,hueMin,satMin,valMin,hueMax,satMax,valMax)
            VALUES (?,?,?,?,?,?,?)
        ''', newColorArray)
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
    def __init__(
        self,
        camera,
        vertFOV,
        horizFOV,
        elevationOfTarget,
        elevationOfCamera,
        angleFromHoriz,
        hsvValues,
        colorDetCon,
        extra_tolerance_factor=0.30,
        on_count_callback=None,
        on_detect_callback=None,
        center_tolerance=0.15,
        base_ignore_above=0.5,
    ):
        self.cap = camera
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.vertFOV = vertFOV
        self.horizFOV = horizFOV
        self.elevationOfTarget = elevationOfTarget
        self.elevationOfCamera = elevationOfCamera
        self.angleFromHoriz = angleFromHoriz
        self.cameraCenter = self.width/2
        self.radiusFromAxisOfRotation = 14/12  # measured in feet (distance from camera to the axis of rotation of the robot)
        self.hsvValues = hsvValues
        self.targetList = {}
        self.masksArray = {}
        self.contours = {}
        self.targetContours = {}
        self.tapeTargetDetected = False
        self.inImg = np.zeros(shape=(self.height, self.width, 3), dtype=np.uint8)
        self.processedImage = self.inImg.copy()
        self.colorDetectConstants = colorDetCon
        self.extraToleranceFactor = extra_tolerance_factor
        self.rescalePercentage = .3
        self.on_count_callback = on_count_callback
        self.on_detect_callback = on_detect_callback
        # Fraction of full frame width defining the "centered" band for a base:
        # centered iff |cx - width/2| <= center_tolerance * width.
        self.centerTolerance = center_tolerance
        # Reject a 'base' whose center is in the upper baseIgnoreAbove fraction of
        # the frame (0.5 = ignore any base in the top half of the view).
        self.baseIgnoreAbove = base_ignore_above

        # Tracking system variables (per camera)
        self.activeTracks = {}  # Dictionary: {color_name: [list of TargetTrack objects]}
        self.nextTrackId = {}  # Dictionary: {color_name: next_id}
        self.frameNumber = 0  # Current frame number
        self.totalCount = {}  # Dictionary: {color_name: count}

        # Tracking parameters (tune these as needed)
        self.MAX_DISTANCE = 150  # Maximum pixel distance for matching (adjust based on your frame size)
        self.MAX_AGE = 5  # Frames to keep track after last detection
        self.ENTRY_ZONE_WIDTH = 0.25  # 15% of frame width for entry detection
        self.MIN_TRACK_AGE = 3  # Minimum frames before counting (prevents false positives)
        self.FONT = cv2.FONT_HERSHEY_SIMPLEX

        # Load detection configurations from database
        self.detectionConfigurations = self.loadDetectionConfigurations()
        self.configToleranceBounds = self.calculateToleranceBounds() if self.detectionConfigurations else None

    def getExpandedTolerance(self, base_tolerance):
        """Expand tolerance by a configurable factor."""
        if base_tolerance <= 0:
            return base_tolerance
        return base_tolerance * (1.0 + self.extraToleranceFactor)

    def isGreenColorName(self, colorName):
        return "green" in colorName.lower()

    def isYellowColorName(self, colorName):
        return "yellow" in colorName.lower()

    def allowedConfigNamesForColor(self, colorName):
        """Limit config matching by color mask source."""
        if self.isGreenColorName(colorName):
            return {"base", "green_1_stalk"}
        if self.isYellowColorName(colorName):
            return {"yellow_1_stalk"}
        return None

    def bboxesOverlap(self, bboxA, bboxB):
        """Return True if two (x, y, w, h) boxes overlap."""
        ax, ay, aw, ah = bboxA
        bx, by, bw, bh = bboxB
        return (
            ax < bx + bw
            and ax + aw > bx
            and ay < by + bh
            and ay + ah > by
        )

    def combineBBoxes(self, bboxA, bboxB):
        """Return bounding box that contains both input boxes."""
        ax, ay, aw, ah = bboxA
        bx, by, bw, bh = bboxB
        x1 = min(ax, bx)
        y1 = min(ay, by)
        x2 = max(ax + aw, bx + bw)
        y2 = max(ay + ah, by + bh)
        return (x1, y1, x2 - x1, y2 - y1)

    @staticmethod
    def _withMeta(detection):
        """Normalize a detection to a (centroid, bbox, meta) triple."""
        if len(detection) >= 3:
            return detection
        centroid, bbox = detection[0], detection[1]
        return (centroid, bbox, {})

    def buildFinalPlantDetections(self, detectionsByConfig):
        """
        Build final classes:
          - base
          - green_1_stalk
          - double_stalk (green + yellow overlap, yellow centroid above green centroid)

        Every detection is a (centroid, bbox, meta) triple. `meta` is a dict that
        rides along to the track. For a double_stalk it records the raw geometry of
        which side the yellow stalk sits on relative to the green stalk center
        ("yellow_side": "left"/"right"); the "which stalk is first" policy is applied
        by the publisher node, not here.
        """
        finalDetections = {
            "base": [self._withMeta(d) for d in detectionsByConfig.get("base", [])],
            "green_1_stalk": [],
            "double_stalk": [],
            "yellow_1_stalk": [],
        }
        yellowDetections = list(detectionsByConfig.get("yellow_1_stalk", []))
        usedYellow = set()

        for greenDetection in detectionsByConfig.get("green_1_stalk", []):
            greenCentroid, greenBBox = greenDetection[0], greenDetection[1]
            matchedYellowIndex = None

            for idx, yellowDetection in enumerate(yellowDetections):
                if idx in usedYellow:
                    continue
                yellowCentroid, yellowBBox = yellowDetection[0], yellowDetection[1]
                if not self.bboxesOverlap(greenBBox, yellowBBox):
                    continue
                if yellowCentroid[1] >= greenCentroid[1]:
                    continue  # yellow must be above green
                matchedYellowIndex = idx
                break

            if matchedYellowIndex is None:
                finalDetections["green_1_stalk"].append(self._withMeta(greenDetection))
                continue

            usedYellow.add(matchedYellowIndex)
            yellowCentroid = yellowDetections[matchedYellowIndex][0]
            yellowBBox = yellowDetections[matchedYellowIndex][1]
            combinedBBox = self.combineBBoxes(greenBBox, yellowBBox)
            combinedCentroid = self.getCentroidFromBBox(combinedBBox)
            # Raw geometry only: which side is the yellow center on relative to green?
            yellowSide = "left" if yellowCentroid[0] < greenCentroid[0] else "right"
            meta = {
                "class": "double_stalk",
                "yellow_side": yellowSide,
                "yellow_center": yellowCentroid,
                "green_center": greenCentroid,
            }
            finalDetections["double_stalk"].append((combinedCentroid, combinedBBox, meta))

        # Yellow stalks that never paired with a green stalk are still real yellow
        # stalks to align on. Emit them as standalone yellow_1_stalk so the
        # yellow_stalk centering signal fires whether or not a double stalk formed;
        # centering falls back to the yellow blob's own centroid (see centeringRefX).
        for idx, yellowDetection in enumerate(yellowDetections):
            if idx in usedYellow:
                continue
            finalDetections["yellow_1_stalk"].append(self._withMeta(yellowDetection))

        return finalDetections

    def readImage(self):

        # this is where the actual video is passed in.
        ret, image = self.cap.read()
        # If frame is read correctly, ret is True
        if not ret or self.inImg is None:
            print("Failed to read from camera 1")
            time.sleep(0.1)
            return False
        self.inImg = cv2.resize(image, (int(self.width*self.rescalePercentage), int(self.height*self.rescalePercentage)), interpolation=cv2.INTER_AREA)
        self.processedImage = self.inImg.copy()
        return True

    def drawBoundingBox(self):
        for configName, target in self.targetContours.items():
            try:
                peri = cv2.arcLength(target, True)
            except Exception:
                print("CV2 Error")
                continue
            approx = cv2.approxPolyDP(target, 0.02 * peri, True)
            x, y, w, h, = cv2.boundingRect(target)
            boundingArea = w * h
            contourArea = cv2.contourArea(target)
            self.targetList[configName] = TapeTarget(self.processedImage, approx, self.tapeTargetDetected, self, (contourArea/boundingArea), configName)

    def processImgForColor(self):
        self.targetList = {}
        self.masksArray = self.getImageMask(self.inImg, self.hsvValues)
        self.contours = self.getContours(self.masksArray)
        self.isolateTarget(self.contours)
        if self.tapeTargetDetected:
            self.drawBoundingBox()
        self.drawTracks(self.processedImage)

    def getContours(self, maskDictionary):
        contourDict = {}
        # Takes the array of masked images and returns an array of countours based off of that.
        for maskName, mask in maskDictionary.items():

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contourDict[maskName] = contours
        return contourDict

    def isolateTarget(self, contourDict):

        idealAreaRatio = self.colorDetectConstants[0]  # this is the ideal ratio for the area ratio value
        areaTolerance = self.colorDetectConstants[1]  # this is the tolerance for finding the target with the right aspect ratio

        idealAspectRatio = self.colorDetectConstants[2]  # this is the ideal aspect ratio based off of the diagram but can be changed as needed.
        aspectTolerance = self.colorDetectConstants[3]
        expandedAreaTolerance = self.getExpandedTolerance(areaTolerance)
        expandedAspectTolerance = self.getExpandedTolerance(aspectTolerance)

        # idealYCoor is the Y coordinate where the target should usually be
        if self.colorDetectConstants[4] == -1:
            idealYCoor = self.height/2
        else:
            idealYCoor = self.colorDetectConstants[4]
        # yCoorTolerance is added and subtracted from the ideal coordinate to create a range on the y axis where the target should be.
        # Any target detected outside that range is ignored
        yCoorTolerance = self.colorDetectConstants[5]

        # idealXCoor is the X coordinate where the target should usually be
        if self.colorDetectConstants[6] == -1:
            idealXCoor = self.width/2
        else:
            idealXCoor = self.colorDetectConstants[6]
        # xCoorTolerance is added and subtracted from the ideal coordinate to create a range on the x axis where the target should be.
        # Any target detected outside that range is ignored
        xCoorTolerance = self.colorDetectConstants[7]

        # start off with a large tolerance, and if the ideal ratio is correct, lower the tolerance as needed.
        self.targetContours = {}
        detectionsByConfig = {
            "base": [],
            "green_1_stalk": [],
            "yellow_1_stalk": [],
        }
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
                        contourArea = cv2.contourArea(contour)  # area of the particle
                        if contourArea <= 20:
                            continue

                        # cv2.findContours returns contours in a specific format

                        # Check if contour has valid points
                        if contour.shape[0] < 3:
                            continue

                        x, y, w, h, = cv2.boundingRect(contour)
                        if w <= 0 or h <= 0:
                            continue

                    except Exception as e:
                        print(f"Invalid contour: {e}")
                        continue
                    boundingArea = w * h
                    # ignores targets that are too small
                    if (boundingArea < 500):
                        continue
                    # ignores targets that are outside a predetermined range

                    self.areaRatio = contourArea/boundingArea
                    self.aspectRatio = w/h

                    # Early exit: Check if outside all configuration tolerances
                    if self.configToleranceBounds:
                        if not self.isWithinAnyTolerance(self.areaRatio, self.aspectRatio):
                            continue  # Skip this contour, it's outside all tolerances

                    # Use scoring system if configurations are loaded
                    if self.detectionConfigurations:
                        allowed_config_names = self.allowedConfigNamesForColor(colorName)
                        best_match = self.getBestConfigurationMatch(
                            self.areaRatio,
                            self.aspectRatio,
                            allowed_config_names=allowed_config_names,
                        )

                        if best_match:
                            config_name, score, within_tolerance = best_match

                            centroid = self.getCentroidFromBBox((x, y, w, h))

                            # A base sits low in the camera view; anything classed
                            # as 'base' in the upper part of the frame is a false
                            # positive -- ignore it entirely (no detection/track).
                            if config_name == "base" and self.isBaseAboveCutoff(centroid):
                                continue

                            largest = contour
                            self.tapeTargetDetected = True

                            if config_name not in detectionsByConfig:
                                detectionsByConfig[config_name] = []
                            detectionsByConfig[config_name].append((centroid, (x, y, w, h)))

                            # Store the best contour for this config type (overwrite if multiple)
                            self.targetContours[config_name] = contour
                    else:
                        # Fallback to original method if no configurations loaded
                        if self.areaRatio > idealAreaRatio - expandedAreaTolerance and self.areaRatio < idealAreaRatio + expandedAreaTolerance:
                            if self.aspectRatio > idealAspectRatio - expandedAspectTolerance and self.aspectRatio < idealAspectRatio + expandedAspectTolerance:
                                largest = contour
                                self.tapeTargetDetected = True

                                centroid = self.getCentroidFromBBox((x, y, w, h))
                                detectionsByConfig["green_1_stalk"].append((centroid, (x, y, w, h)))

                                self.targetContours["green_1_stalk"] = contour

        finalDetections = self.buildFinalPlantDetections(detectionsByConfig)
        if finalDetections["double_stalk"]:
            self.tapeTargetDetected = True
            # Use a representative contour for drawing/labels.
            if "green_1_stalk" in self.targetContours:
                self.targetContours["double_stalk"] = self.targetContours["green_1_stalk"]

        self.updateTracks(finalDetections, self.frameNumber)
        self.frameNumber += 1

    def getImageMask(self, img, myColors):
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        masks = {}
        for colorName, color in myColors.items():

            lower = np.array(color[0:3], dtype=np.uint8)
            upper = np.array(color[3:6], dtype=np.uint8)
            masks[colorName] = cv2.inRange(imgHSV, lower, upper)

        return masks

    def getCentroidFromBBox(self, bbox):
        """Calculate centroid from bounding box (x, y, w, h)"""
        x, y, w, h = bbox
        return (x + w // 2, y + h // 2)

    def isBaseAboveCutoff(self, centroid):
        """True if a base centroid is too high in the frame to be a real base.

        Image y grows downward (0 = top). A real base sits low in view, so a base
        whose center is above baseIgnoreAbove * frameHeight (measured from the top)
        is rejected. baseIgnoreAbove = 0.5 rejects the entire upper half.
        """
        frameHeight = self.processedImage.shape[0]
        return centroid[1] < self.baseIgnoreAbove * frameHeight

    def isCentered(self, cx, frameWidth):
        """True if an x coordinate sits within the centered band of the frame.

        The band is [center - tol*width, center + tol*width] where tol is
        self.centerTolerance (a fraction of the full frame width). e.g. tol=0.15
        means the middle 30% of the frame counts as centered.
        """
        center = frameWidth / 2.0
        return abs(cx - center) <= self.centerTolerance * frameWidth

    def centeringRefX(self, configName, track):
        """X coordinate used to decide whether a track is 'centered'.

        A double stalk is aligned by the yellow stalk itself (the piece to be
        removed), whose center is carried in the track meta; everything else uses
        the track's own centroid. A standalone yellow_1_stalk therefore aligns on
        its own centroid, which is exactly the yellow stalk's center.
        """
        if configName == "double_stalk":
            yellowCenter = (track.meta or {}).get("yellow_center")
            if yellowCenter is not None:
                return yellowCenter[0]
        return track.centroid[0]

    def loadDetectionConfigurations(self):
        """Load detection configurations from database recommendations"""
        data_dir = _get_vision_data_dir()
        db_path = os.path.join(data_dir, "hsv.db")

        if not os.path.exists(db_path):
            print("Warning: Database not found. Using default colorDetectConstants.")
            return None

        try:
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()

            # Define which sessions map to which object types
            session_mappings = {
                'base': 'CornZeroStalk',
                'green_1_stalk': 'CornGreen1Stalk',
                'yellow_1_stalk': 'CornYellow1Stalk'
            }

            configurations = []
            for obj_type, session_name in session_mappings.items():
                cursor.execute('''
                    SELECT areaRatio_ideal, areaRatio_tolerance,
                           aspectRatio_ideal, aspectRatio_tolerance
                    FROM detection_recommendations
                    WHERE session_name = ?
                    ORDER BY timestamp DESC
                    LIMIT 1
                ''', (session_name,))

                result = cursor.fetchone()
                if result:
                    configurations.append({
                        'name': obj_type,
                        'idealAreaRatio': result[0],
                        'areaTolerance': result[1],
                        'idealAspectRatio': result[2],
                        'aspectTolerance': result[3]
                    })
                    print(f"Loaded configuration for {obj_type} from session '{session_name}'")
                else:
                    print(f"Warning: No recommendations found for {obj_type} (session: {session_name})")

            conn.close()

            if len(configurations) == 0:
                print("No configurations loaded. Falling back to default constants.")
                return None

            return configurations

        except Exception as e:
            print(f"Error loading detection configurations from database: {e}")
            return None

    def calculateToleranceBounds(self):
        """Calculate min/max bounds for all configurations to enable early exit"""
        if not self.detectionConfigurations:
            return None

        min_area_ratio = float('inf')
        max_area_ratio = float('-inf')
        min_aspect_ratio = float('inf')
        max_aspect_ratio = float('-inf')

        for config in self.detectionConfigurations:
            expanded_area_tolerance = self.getExpandedTolerance(config['areaTolerance'])
            expanded_aspect_tolerance = self.getExpandedTolerance(config['aspectTolerance'])
            area_min = config['idealAreaRatio'] - expanded_area_tolerance
            area_max = config['idealAreaRatio'] + expanded_area_tolerance
            aspect_min = config['idealAspectRatio'] - expanded_aspect_tolerance
            aspect_max = config['idealAspectRatio'] + expanded_aspect_tolerance

            min_area_ratio = min(min_area_ratio, area_min)
            max_area_ratio = max(max_area_ratio, area_max)
            min_aspect_ratio = min(min_aspect_ratio, aspect_min)
            max_aspect_ratio = max(max_aspect_ratio, aspect_max)

        return {
            'min_area_ratio': min_area_ratio,
            'max_area_ratio': max_area_ratio,
            'min_aspect_ratio': min_aspect_ratio,
            'max_aspect_ratio': max_aspect_ratio
        }

    def isWithinAnyTolerance(self, areaRatio, aspectRatio):
        """Quick check if detection is within any configuration's tolerance (early exit)"""
        if not self.configToleranceBounds:
            return True  # If no bounds, don't filter

        bounds = self.configToleranceBounds
        return (bounds['min_area_ratio'] <= areaRatio <= bounds['max_area_ratio'] and
                bounds['min_aspect_ratio'] <= aspectRatio <= bounds['max_aspect_ratio'])

    def calculateConfigurationScore(self, areaRatio, aspectRatio, idealAreaRatio, idealAspectRatio, areaTolerance, aspectTolerance):
        """
        Calculate a normalized squared distance score for how well a detection matches a configuration.
        Lower score = better match. Uses squared distance (no sqrt) for performance.
        """
        # Normalize differences by their tolerances (so both dimensions are on similar scale)
        expanded_area_tolerance = self.getExpandedTolerance(areaTolerance)
        expanded_aspect_tolerance = self.getExpandedTolerance(aspectTolerance)

        if expanded_area_tolerance > 0:
            normalized_area_diff = (areaRatio - idealAreaRatio) / expanded_area_tolerance
        else:
            normalized_area_diff = abs(areaRatio - idealAreaRatio) if idealAreaRatio > 0 else 1.0

        if expanded_aspect_tolerance > 0:
            normalized_aspect_diff = (aspectRatio - idealAspectRatio) / expanded_aspect_tolerance
        else:
            normalized_aspect_diff = abs(aspectRatio - idealAspectRatio) if idealAspectRatio > 0 else 1.0

        # Calculate squared distance in normalized space (no sqrt for performance)
        distance_squared = normalized_area_diff**2 + normalized_aspect_diff**2

        # Check if within tolerance
        within_tolerance = (
            idealAreaRatio - expanded_area_tolerance <= areaRatio <= idealAreaRatio + expanded_area_tolerance
            and idealAspectRatio - expanded_aspect_tolerance <= aspectRatio <= idealAspectRatio + expanded_aspect_tolerance
        )

        # If outside tolerance, add penalty
        if not within_tolerance:
            distance_squared += 100.0  # Large penalty for being outside tolerance

        return distance_squared, within_tolerance

    def scoreAgainstConfigurations(self, areaRatio, aspectRatio, allowed_config_names=None):
        """
        Score a detection against all loaded configurations.

        Returns:
            List of tuples: (config_name, score, within_tolerance) sorted by score (best first)
        """
        if not self.detectionConfigurations:
            return []

        scores = []
        for config in self.detectionConfigurations:
            if allowed_config_names and config["name"] not in allowed_config_names:
                continue
            score, within_tolerance = self.calculateConfigurationScore(
                areaRatio, aspectRatio,
                config['idealAreaRatio'],
                config['idealAspectRatio'],
                config['areaTolerance'],
                config['aspectTolerance']
            )
            scores.append((config['name'], score, within_tolerance))

        # Sort by score (lower is better)
        scores.sort(key=lambda x: x[1])
        return scores

    def getBestConfigurationMatch(self, areaRatio, aspectRatio, allowed_config_names=None):
        """
        Get the best matching configuration for a detection.

        Returns:
            Tuple: (best_config_name, best_score, within_tolerance) or None if no good match
        """
        scores = self.scoreAgainstConfigurations(
            areaRatio,
            aspectRatio,
            allowed_config_names=allowed_config_names,
        )

        if not scores:
            return None

        best_name, best_score, within_tolerance = scores[0]

        # Only return if score is reasonable (within tolerance or very close)
        # Score threshold: 2.0 means within ~1.4 normalized units (sqrt(2))
        if within_tolerance or best_score < 2.0:
            return (best_name, best_score, within_tolerance)

        return None

    def matchDetectionsToTracks(self, detections, colorName, frameNumber):
        """
        Match new detections to existing tracks using nearest neighbor approach.
        detections: list of (centroid, bbox) tuples
        color_name: name of the color being tracked
        Returns: (matched_pairs, unmatched_detections, unmatched_tracks)
        """
        # Initialize tracking structures for this color if needed
        if colorName not in self.activeTracks:
            self.activeTracks[colorName] = []
        if colorName not in self.nextTrackId:
            self.nextTrackId[colorName] = 1
        if colorName not in self.totalCount:
            self.totalCount[colorName] = 0

        tracks = self.activeTracks[colorName]

        if len(tracks) == 0:
            # No existing tracks, all detections are new
            return [], detections, []
        if len(detections) == 0:
            # No detections, all tracks are unmatched
            return [], [], tracks.copy()

        distanceMatrix = []
        for track in tracks:
            row = []
            for det in detections:
                row.append(track.getDistanceSquared(det[0]))
            distanceMatrix.append(row)

        # Greedy matching: match closest pairs within threshold
        matchedPairs = []
        matchedTrackIndices = set()
        matchedDetIndices = set()

        # Sort all possible matches by distance
        allMatches = []

        for tIDx, track in enumerate(tracks):
            for dIDx, det in enumerate(detections):
                distanceSQ = distanceMatrix[tIDx][dIDx]
                MAX_DIST_SQ = self.MAX_DISTANCE * self.MAX_DISTANCE
                if distanceSQ <= MAX_DIST_SQ:
                    allMatches.append((distanceSQ, tIDx, dIDx))
        allMatches.sort(key=lambda x: x[0])

        for distanceSQ, tIDx, dIDx in allMatches:
            if tIDx not in matchedTrackIndices and dIDx not in matchedDetIndices:
                matchedPairs.append((tracks[tIDx], detections[dIDx]))
                matchedTrackIndices.add(tIDx)
                matchedDetIndices.add(dIDx)

        unmatchedTracks = [tracks[i] for i in range(len(tracks)) if i not in matchedTrackIndices]
        unmatchedDetections = [detections[i] for i in range(len(detections)) if i not in matchedDetIndices]

        return matchedPairs, unmatchedDetections, unmatchedTracks

    def checkEntryZone(self, track, frameWidth):
        """
        Check if a track has entered the counting zone and should be counted.
        Returns True if track should be counted.
        """

        if track.counted:
            return False
        if track.age < self.MIN_TRACK_AGE:
            return False
        # Define entry zone (left side of frame)
        entryThreshold = frameWidth * self.ENTRY_ZONE_WIDTH

        if track.centroid[0] <= entryThreshold:
            if len(track.history) > 1:
                prevX = track.history[-2][0]
                if prevX > entryThreshold:
                    return True
        return False

    def updateTracks(self, detectionsByColor, frameNumber):
        """
        Main tracking update function.
        detections_by_color: dictionary {config_name: list of (centroid, bbox) tuples}
        Note: Uses final class names (base, green_1_stalk, double_stalk).
        """
        frame_width = self.processedImage.shape[1]

        for configName, detections in detectionsByColor.items():
            matchedPairs, unmatchedDetections, unmatchedTracks = \
                self.matchDetectionsToTracks(detections, configName, frameNumber)

            for track, det in matchedPairs:
                centroid, bbox = det[0], det[1]
                meta = det[2] if len(det) > 2 else {}
                track.update(centroid, bbox, frameNumber, meta)
                # Announce once, the moment the object is confirmed on screen.
                # This is independent of the counting line below: it fires as soon
                # as the track is stable (>= MIN_TRACK_AGE frames), so downstream
                # nodes learn "what am I seeing right now" the instant it enters.
                centeringClass = configName in ("base", "double_stalk", "yellow_1_stalk")
                if not track.announced and track.age >= self.MIN_TRACK_AGE:
                    track.announced = True
                    announceMeta = dict(track.meta or {})
                    if centeringClass:
                        track.centeredState = self.isCentered(
                            self.centeringRefX(configName, track), frame_width)
                        announceMeta["centered"] = track.centeredState
                    if self.on_detect_callback:
                        self.on_detect_callback(configName, announceMeta)
                # A base (stop to seed) and a double stalk's yellow stalk (stop to
                # remove it) are stop/align targets: after entry, keep subscribers
                # updated whenever the object crosses into or out of the centered
                # zone. Only re-announced on an actual state change.
                elif centeringClass and track.announced:
                    centered = self.isCentered(
                        self.centeringRefX(configName, track), frame_width)
                    if centered != track.centeredState:
                        track.centeredState = centered
                        if self.on_detect_callback:
                            announceMeta = dict(track.meta or {})
                            announceMeta["centered"] = centered
                            self.on_detect_callback(configName, announceMeta)
                # check if this track should be counted. Standalone yellow stalks
                # drive the yellow_stalk align signal but are not part of the
                # S/D/E tally, so they are announced (above) but never counted.
                if configName != "yellow_1_stalk" and self.checkEntryZone(track, frame_width):
                    track.counted = True
                    # Initialize count if needed
                    if configName not in self.totalCount:
                        self.totalCount[configName] = 0
                    self.totalCount[configName] += 1
                    if self.on_count_callback:
                        self.on_count_callback(configName, dict(self.totalCount))
                    print(f"[{configName}] Target #{self.totalCount[configName]} counted! Track ID: {track.trackID}")
            for det in unmatchedDetections:
                centroid, bbox = det[0], det[1]
                meta = det[2] if len(det) > 2 else {}
                newTrack = TargetTrack(
                    self.nextTrackId[configName] if configName in self.nextTrackId else 0,
                    centroid,
                    bbox,
                    frameNumber,
                    configName,
                    meta,
                )
                # Initialize tracking structures if needed
                if configName not in self.activeTracks:
                    self.activeTracks[configName] = []
                if configName not in self.nextTrackId:
                    self.nextTrackId[configName] = 0
                self.activeTracks[configName].append(newTrack)
                self.nextTrackId[configName] += 1

            if configName in self.activeTracks:
                self.activeTracks[configName] = [
                    track for track in self.activeTracks[configName]
                    if (frameNumber - track.lastSeen) <= self.MAX_AGE
                ]

    def drawTracks(self, img):
        frameWidth = img.shape[1]
        entryX = int(frameWidth*self.ENTRY_ZONE_WIDTH)
        text_scale = 0.4
        text_thickness = 1

        # Draw entry zone line
        cv2.line(img, (entryX, 0), (entryX, img.shape[0]), (255, 0, 255), 2)
        cv2.putText(img, "ENTRY ZONE", (entryX + 5, 30), self.FONT, text_scale, (255, 0, 255), text_thickness)

        # Only draw tracks that were seen recently (within 1 frame) to prevent stale bounding boxes
        RECENT_FRAMES = 1  # Only show bounding boxes for tracks seen in current or previous frame

        # draw tracks for each config type
        for configName, tracks in self.activeTracks.items():
            if configName == "base":
                trackColor = (0, 255, 0)  # Green for base
            elif configName == "green_1_stalk":
                trackColor = (0, 255, 255)  # Cyan for green 1 stalk
            elif configName == "double_stalk":
                trackColor = (255, 0, 255)  # Magenta for double stalk
            else:
                trackColor = (255, 0, 0)  # Blue for other types

            for track in tracks:
                # Only draw if track was seen recently (within RECENT_FRAMES)
                frames_since_seen = self.frameNumber - track.lastSeen
                if frames_since_seen > RECENT_FRAMES:
                    continue  # Skip drawing stale tracks

                x, y, w, h = track.bbox
                color = trackColor if not track.counted else (0, 165, 255)  # Orange if counted

                # Draw bounding box (tracks seen in current frame get full opacity)
                cv2.rectangle(img, (x, y), (x+w, y+h), color, 2)

                # Draw centroid
                cx, cy = track.centroid
                cv2.circle(img, (cx, cy), 5, color, -1)

                # Draw Track ID and count status
                label = f"{configName} ID:{track.trackID}"
                if track.counted:
                    label += "(COUNTED)"

                cv2.putText(img, label, (x, y - 10), self.FONT, text_scale, color, text_thickness)

                if len(track.history) >= 1:
                    points = np.array(track.history, np.int32)
                    cv2.polylines(img, [points], False, color, 2)
        yOffset = 30
        for configName, count in self.totalCount.items():
            countText = f"{configName} Count: {count}"
            cv2.putText(img, countText, (10, yOffset), self.FONT, text_scale, (0, 255, 255), text_thickness)
            yOffset += 25


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
        # (height of target [feet] - height of camera [feet])/tan(pitch [degrees] + angle of camera [degrees])
        self.distanceToTarget = (camera.elevationOfTarget - camera.elevationOfCamera) / math.tan(math.radians(self.pitch + camera.angleFromHoriz))
        self.targetColor = targetC

    def drawRectangle(self, imageResult):
        # Draw rectangle on the Image
        cv2.rectangle(imageResult, (self.x, self.y), (self.x+self.w, self.y+self.h), (0, 255, 0), 3)
        cv2.putText(imageResult, self.targetColor, (self.x+20, self.y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)


class TargetTrack(object):
    """Represents a tracked target across multiple frames"""
    def __init__(self, track_id, centroid, bbox, frameNumber, color_name, meta=None):
        self.trackID = track_id
        self.centroid = centroid  # (cx, cy) tuple
        self.bbox = bbox  # (x, y, w, h) tuple
        self.age = 1  # Number of frames this track has existed
        self.lastSeen = frameNumber  # Frame number when last updated
        self.counted = False  # Whether this target has been counted
        self.announced = False  # Whether this object's on-screen entry was announced
        self.centeredState = None  # base only: last announced centered-ness (bool)
        self.entrySide = None  # 'left', 'right', or None - which side it entered from
        self.history = [centroid]  # History of centroid positions for visualization
        self.colorName = color_name  # Color name for this track
        self.meta = meta or {}  # Extra per-object info (e.g. double_stalk yellow_side)

    def update(self, centroid, bbox, frameNumber, meta=None):
        """Update track with new detection"""
        self.centroid = centroid
        self.bbox = bbox
        self.age += 1
        self.lastSeen = frameNumber
        self.history.append(centroid)
        if meta:
            self.meta = meta
        # Keep only last 10 positions for visualization
        if len(self.history) > 10:
            self.history.pop(0)

    def getDistanceSquared(self, other_centroid):
        """Calculate Euclidean distance to another centroid"""
        return (self.centroid[0] - other_centroid[0])**2 + \
            (self.centroid[1] - other_centroid[1])**2


class VisionApplication(object):
    def __init__(
        self,
        use_pi_camera=True,
        camera_index=0,
        camera2_index=1,
        camera_width=640,
        camera_height=480,
        extra_tolerance_factor=0.20,
        on_count_callback=None,
        on_detect_callback=None,
        center_tolerance=0.15,
        base_ignore_above=0.5,
        streaming_video=False,
        stream_port=8080,
        use_gui=False,
    ):
        self.cameraInUse = 1
        self.use_pi_camera = use_pi_camera
        self.camera_index = camera_index
        self.camera2_index = camera2_index
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.extra_tolerance_factor = extra_tolerance_factor
        self.on_count_callback = on_count_callback
        self.on_detect_callback = on_detect_callback
        self.center_tolerance = center_tolerance
        self.base_ignore_above = base_ignore_above
        self.streaming_video = streaming_video
        self.stream_port = stream_port
        self.use_gui = use_gui

        # Set Number of Cameras
        self.numberOfCameras = 1

        self.hsvDataBase = DataBase("hsv")
        self.myColors = self.hsvDataBase.readhsvValues()

        self.areaRatio = 0
        self.largestAreaRatio = 0
        self.aspectRatio = 0
        self.largestAspectRatio = 0

        # More reasonable detection constants for general object detection
        # [idealAreaRatio, areaTolerance, idealAspectRatio, aspectTolerance, idealY, yTolerance, idealX, xTolerance]
        colorDetectConstants = [.6, .4, .3, 4, 90, 200, -1, 200]
        self.contours = None

        self.running = True

        if self.streaming_video:
            # Start server in a separate thread so it doesn't block
            self.server = HTTPServer(('0.0.0.0', self.stream_port), StreamingHandler)
            server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            server_thread.start()
            print(f"Streaming server started on http://0.0.0.0:{self.stream_port}/stream")

        # Distance Calculation Constants
        vertFOV = [48.94175846, 1]
        horizFOV = [134.3449419, 1]
        elevationOfTarget = [1.5, 1]
        elevationOfCamera = [0.9, 1]
        angleFromHoriz = [30, 1]

        posColorVals = {}
        IDColorVals = {}
        for color, values in self.myColors.items():
            color_lower = color.lower()
            if "green" in color_lower or "yellow" in color_lower:
                posColorVals[color] = values
            else:
                IDColorVals[color] = values
        if len(posColorVals) == 0:
            print("Warning: No green/yellow HSV colors found in DB for plant detection.")
        self.cameraList = {}
        camera_source_1 = create_camera_capture(
            use_pi_camera=self.use_pi_camera,
            camera_index=self.camera_index,
            width=self.camera_width,
            height=self.camera_height,
        )
        self.camera = CameraView(
            camera_source_1,
            vertFOV[0],
            horizFOV[0],
            elevationOfTarget[0],
            elevationOfCamera[0],
            angleFromHoriz[0],
            posColorVals,
            colorDetectConstants,
            extra_tolerance_factor=self.extra_tolerance_factor,
            on_count_callback=self.on_count_callback,
            on_detect_callback=self.on_detect_callback,
            center_tolerance=self.center_tolerance,
            base_ignore_above=self.base_ignore_above,
        )
        self.cameraList["PositioningCamera"] = self.camera
        if self.numberOfCameras == 2:
            camera_source_2 = create_camera_capture(
                use_pi_camera=self.use_pi_camera,
                camera_index=self.camera2_index,
                width=self.camera_width,
                height=self.camera_height,
            )
            self.camera2 = CameraView(
                camera_source_2,
                vertFOV[1],
                horizFOV[1],
                elevationOfTarget[1],
                elevationOfCamera[1],
                angleFromHoriz[1],
                IDColorVals,
                colorDetectConstants,
                extra_tolerance_factor=self.extra_tolerance_factor,
                on_count_callback=self.on_count_callback,
                on_detect_callback=self.on_detect_callback,
                center_tolerance=self.center_tolerance,
                base_ignore_above=self.base_ignore_above,
            )
            self.cameraList["IDCamera"] = self.camera2

    def cameraThread(self):

        print("Camera thread started")
        frame_skip = 0
        frame_counter = 0
        while self.running:
            frame_counter += 1
            for camName, cam in self.cameraList.items():
                if not cam.readImage():
                    continue
                if frame_skip > 0 and frame_counter % (frame_skip + 1) != 0:
                    continue
                cam.processImgForColor()
                if cam.tapeTargetDetected:
                    for configName, target in cam.targetList.items():
                        target.drawRectangle(cam.processedImage)
                        # Print aspect ratio (and area ratio) instead of coords —
                        # these are what the classifier tolerances match on, so this
                        # shows why a green_1_stalk may be read as a base.
                        print(f"{configName}: aspect={target.aspectRatio:.2f} area={target.areaRatio:.2f}", end=' | ')
                    print()

                scale_factor = 1

                for maskName, mask in cam.masksArray.items():
                    new_width = int(mask.shape[1] * scale_factor)
                    new_height = int(mask.shape[0] * scale_factor)
                    resizedMask = cv2.resize(mask, (new_width, new_height))

                new_width = int(cam.processedImage.shape[1] * scale_factor)
                new_height = int(cam.processedImage.shape[0] * scale_factor)
                resizedResult = cv2.resize(cam.processedImage, (new_width, new_height))

                if self.streaming_video:
                    with StreamingHandler.frame_lock:
                        StreamingHandler.latest_frame = resizedResult.copy()

            if self.use_gui:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Quit key pressed")
                    self.running = False
                    break

            time.sleep(0.1)

    def runApplication(self):
        print("Starting application...")

        try:
            self.cameraThread()
        except KeyboardInterrupt:
            print("Application interrupted by user")
        finally:
            self.hsvDataBase.commitAndClose()
            print("Cleaning up...")
            self.running = False
            for cam in self.cameraList.values():
                if hasattr(cam, "cap") and cam.cap:
                    cam.cap.release()
            if self.use_gui:
                cv2.destroyAllWindows()
            print("Application ended")
