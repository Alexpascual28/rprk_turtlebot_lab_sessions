# Machine vision lab - Practical Robotics
# test_camera.py: Grab the camera framebuffer in OpenCV and display using cv2.imshow


# Import the necessary packages
import picamera
import picamera.array
import time
import cv2

# Initialise the camera and create a reference to it
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = picamera.array.PiRGBArray(camera, size=camera.resolution)

# Allow the camera time to warm up
time.sleep(0.1)

# Create counter for FPS
frame_count = 0
start_time = time.time()

# Capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

    frame_count += 1
    average_fps = frame_count / ( time.time() - start_time )
    cv2.putText(image,"%2.1f fps" % average_fps, (50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2,cv2.LINE_AA)
    # Show the frame
    cv2.imshow("Frame", image)

    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
