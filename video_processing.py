import cv2

def main():
    # get the camera feed
    camera_feed = cv2.VideoCapture(1)

    # output to the screen
    while(camera_feed.isOpened()) :
        is_capturing_feed, cur_frame = camera_feed.read()
        if is_capturing_feed:
            cv2.imshow("Frame", cur_frame)

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