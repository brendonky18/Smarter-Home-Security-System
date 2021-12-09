import cv2
import numpy as np

from imutils.object_detection import non_max_suppression as nms

FRONT_FACE_CASCADE = "haar_cascades/haarcascade_frontalface_default.xml"
PROFILE_FACE_CASCADE = "haar_cascades/haarcascade_profileface.xml"

def cascade_detect(image, cascade):
    grayscale_frame = cv2.equalizeHist(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))

    detected_objects = cascade.detectMultiScale(grayscale_frame)

    return [(x_coord, y_coord, x_coord+width, y_coord+height) for (x_coord, y_coord, width, height) in detected_objects]

    # for (x_coord, y_coord, width, height) in detected_objects:

    # pass

def render_frame(frame):

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
    HOG_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # get the camera feed
    camera_feed = cv2.VideoCapture(1)

    # output to the screen
    while(camera_feed.isOpened()) :
        is_capturing_feed, cur_frame = camera_feed.read()
        if is_capturing_feed:
            # front_faces = cascade_detect(cur_frame, front_face_cascade)
            profile_faces = cascade_detect(cur_frame, profile_face_cascade)

            people, weights = HOG_detector.detectMultiScale(cur_frame)

            people = nms(np.array([(x_coord, y_coord, x_coord+width, y_coord+height) for (x_coord, y_coord, width, height) in people]), overlapThresh=0.7)

            drawn_frame = cur_frame

            # print(type(profile_faces))
            # print(type(people))

            for (xA, yA, xB, yB) in profile_faces:
                drawn_frame = cv2.rectangle(drawn_frame, (xA, yA), (xB, yB), (0,0,255))

            for (xA, yA, xB, yB) in people:
                drawn_frame = cv2.rectangle(drawn_frame, (xA, yA), (xB, yB), (0,255,0))

            render_frame(drawn_frame)

            # press q to close the program
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        else:
            break

    # clean up
    camera_feed.release()
    cv2.destroyAllWindows()


if __name__ == "__main__": 
    main()