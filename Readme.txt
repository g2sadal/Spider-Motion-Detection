Raspberry Pi Multi-Camera UI Instructions
1. Setup
Download the Ui.zip archive from Pucheng’s Dropbox.


Unzip it to get a folder named UI/, which contains:


ui.exe: The main program.


IP.txt: A text file listing the IP addresses of the Raspberry Pi devices to connect.


2. Configure Raspberry Pi Connections
On each Raspberry Pi, run the command hostname -I to check its IP address.


Open the IP.txt file and add the IP addresses of the Raspberry Pis you want to connect, one per line. Example:


pi1 192.168.1.101
pi2 192.168.1.102
Save the file. The UI will read this file and connect to the listed devices.


3. UI Button Functions
START
Starts the motion detection program across all listed Raspberry Pis.


Begins capturing frames based on settings.


The button below allows you to select a local folder for saving the output (default is Desktop).


STOP
Stops the program on all Raspberry Pis.


Downloads all generated files from each Pi, including recorded videos, timestamps, and more.


PAIR
Synchronizes frames from different Raspberry Pis.


！！Before clicking this, manually gather all the .json timestamp files into one single folder.


CAPTURE
Takes one still photo from each Raspberry Pi simultaneously.


Photos are saved locally on each Raspberry Pi.


4. Settings Panel
Setting	Description
Resolution	Image resolution (e.g., 4608×2592). Adjust according to Pi performance.
Num of stored frames	Number of previous frames stored in a queue. When motion is detected, these past frames are also saved. Prevents frame loss due to fast movement. Recommended: 5–10.
Sec per frame	Time interval (in seconds) between consecutive frames. Acts as the inverse of frame rate. Smaller values = higher FPS.
Stop delay (s)	Delay (in seconds) before stopping recording after no motion is detected. Recommended range: 1–3 seconds.
Motion threshold	Threshold for motion detection, defined as the proportion of changed pixels. Higher value = more significant motion required. Suggested range: 0.005–0.02.


5. Pairing Parameters (below STOP button)
Setting	Description
Threshold (s)	Time difference threshold to determine if two frames are close enough to be considered a pair.
Interval (s)	Expected time interval between frames. Should be equal to 'Sec per frame' set above.


6. Device Status
The “Pi state” section on the lower right shows connection status of each Raspberry Pi (e.g., pi1, pi2, etc.).


This helps you quickly verify if all devices are connected and responsive.


If you encounter issues or need help, please contact the developer: pshao7@jh.edu
