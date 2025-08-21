from picamera2 import Picamera2

picam0 = Picamera2(0)
c = picam0.create_still_configuration({"size": (4608,2592)})
picam0.configure(c)

picam0.start_and_capture_file("calibr_cam0_pi1.jpg")
picam0.stop()

picam1 = Picamera2(1)
c = picam1.create_still_configuration({"size": (4608,2592)})
picam1.configure(c)

picam1.start_and_capture_file("calibr_cam1_pi1.jpg")
picam1.stop()

