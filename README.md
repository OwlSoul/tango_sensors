# Tango ROS node for Android (Yellowstone tablet)

APK: https://github.com/YuryXW/tango_sensors/blob/master/release/tango_sensors-debug.apk?raw=true

Application to publish data from Google Tango device to ROS (Robotic Operating System).
Was designed and is guaranteed to run only on Yellowstone tablet with Kitkat Android.

Unlike "tango_ros_streamer", this app is designed to publish raw data only, and only for
Tango related stuff, i.e. Pose, Point Cloud and video stream from both cameras (Color and Fisheye).
No IMU/Navigation/Sensor e.t.c data is being published.

App is designed to provide data as fast as possible, which creates a bottleneck when using WiFi connection.
Limiting publish rate for images and point cloud in case you use Wireless connection (can be done in app dashboard),
or using wired connection to ROS master (USB Tethering) is recommended. Note that Point Cloud message is the most resource-consuming, taking up to 5-6 MB/s when there's a lot of points. Images at high resolution will also take significant part of your bandwidth. Play up with settings to reduce publish rate and/or image size and compression to reduce bottleneck effect.

Current acheived rates for message publishing are:
- Up to 100 HZ for Pose (technical limit of Tango pose estimation).
- Up to 5 HZ for Point Cloud (technical limit of Tango point cloud calculation)
- Up to 30 HZ for both imagess, but generally lower, heavily dependent on resolution you've chosen.
  For 320x240 image publish rate is about 15-18HZ. All images are being compressed to JPEG before publishing.
  Using all sensors together will reduce publish rate for about 50% on WiFi. 

Known bugs and limitations:
- App will not tell you if it didn't connect to ROS master or not, use ROS tools to check this (technically a feature).
- It is not adviseable to launch this app on startup immediately, it may crash. Wait until all your apps are started.
- ROI (region of interest) is not supported for cameras.
- Your Yellowstone tablet will become REALLY hot and will drain power pretty fast when you're using all four Tango
  related publishers at max speed. Interface also may become "a little bit" unresponsive, accept our apologies for that.
- There's a really small chance Tango will not start automatically in case it crashed before, switching something (like pose or 
  point cloud) on and off will fix this.
- Avoid low battery level (network operations will become slow and buggy), it's adviseable that if you use this app on your 
  robot, you power Yellowstone tablet off somehow with at least 2 AMPS of power.
  
It is recommended to use apk file provided: 

https://github.com/YuryXW/tango_sensors/blob/master/release/tango_sensors-debug.apk?raw=true

In case you want to assemble app by yourselves, go on a good and active weekend first
to recharge your batteries. Creating ROS apps for Android may take all your sanity in one day.
  
I do not own Google Tango on permanent basis, so this app can be considered "unmaintained". Application is provided AS IS with
no guarantees, and is not recommended for production usage. Feel free to use it for personal or academic purposes.

Refer to wiki section for learning how to work with this application.
