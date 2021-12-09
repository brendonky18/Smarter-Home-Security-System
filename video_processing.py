import cv2
import numpy as np

from imutils.object_detection import non_max_suppression as nms
from time import time_ns as nanoseconds

FRONT_FACE_CASCADE = "haar_cascades/haarcascade_frontalface_default.xml"
PROFILE_FACE_CASCADE = "haar_cascades/haarcascade_profileface.xml"
DETECTION_PERIOD = 500

def millis():
    return nanoseconds() // 1000000

def detect(image, detector):
    grayscale_frame = cv2.equalizeHist(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))

    results = detector.detectMultiScale(grayscale_frame)

    detected_objects = results
    
    if isinstance(detector, cv2.HOGDescriptor):      # HOG
        detected_objects = results[0]
    elif isinstance(detector, cv2.CascadeClassifier):                                           # Cascade
        detected_objects = results
    else:
        raise RuntimeError("Can only handle detectors that are HOGs or Haar Cascades")

    return nms(np.array([(x_coord, y_coord, x_coord+width, y_coord+height) for (x_coord, y_coord, width, height) in detected_objects]), overlapThresh=0.7)

def render_frame(frame, bounding_boxes):
    for (xA, yA, xB, yB) in bounding_boxes:
        cv2.rectangle(frame, (xA, yA), (xB, yB), (0,0,255))

    cv2.imshow("Frame", frame)

def main():
    #  init cascades
    front_face_cascade = cv2.CascadeClassifier()
    profile_face_cascade = cv2.CascadeClassifier()

    if not front_face_cascade.load(cv2.samples.findFile(FRONT_FACE_CASCADE)):
        raise RuntimeError("Could not load front face cascade")

    if not profile_face_cascade.load(cv2.samples.findFile(PROFILE_FACE_CASCADE)):
        raise RuntimeError("Could not load profile face cascade")

    # init Histogram of Oriented Gradients (HOG)
    HOG_detector = cv2.HOGDescriptor()
    HOG_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector(), )

    # init detectors
    detectors = [front_face_cascade, profile_face_cascade, HOG_detector]
    next_detector = 0

    # init tracker
    multiTrackers = [cv2.legacy.MultiTracker_create() for i in detectors]

    # init time
    cur_time = millis()
    detection_time = cur_time - 1

    # init the camera feed
    camera_feed = cv2.VideoCapture(1)

    # output to the screen
    while(camera_feed.isOpened()) :
        is_capturing_feed, cur_frame = camera_feed.read()
        if not is_capturing_feed:
            break
        
        # press q to close the program
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
        
        # distribute each of the dectors over the entire period 
        # so we're not performing several large calculations simultaneously
        if cur_time >= (detection_time + DETECTION_PERIOD/len(detectors)):
            detected_objects = detect(cur_frame, detectors[next_detector])
            
            # get rid of old trackers so we don't have duplicates, recreate tracker
            multiTrackers[next_detector] = cv2.legacy.MultiTracker_create()
            
            # assign objects to tracker
            for object in detected_objects:
                multiTrackers[next_detector].add(cv2.legacy.TrackerKCF_create(), cur_frame, object)

            detection_time = int(DETECTION_PERIOD/len(detectors) + cur_time)
            next_detector = (next_detector + 1) % len(detectors)

        cur_time = millis()

        # update trackers
        tracked_objects = []
        for (successes, cur_tracked_objects) in [multiTracker.update(cur_frame) for multiTracker in multiTrackers]:
            tracked_objects += [(int(x_coord), int(y_coord), int(x_coord+width), int(y_coord+height)) for (x_coord, y_coord, width, height) in cur_tracked_objects]

        render_frame(cur_frame, tracked_objects)

    # clean up
    camera_feed.release()
    cv2.destroyAllWindows()

if __name__ == "__main__": 
    main()