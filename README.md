# NetworkTables
Creating a network tables bridge from ROS2 to WPILib

---

# Running Multiple AprilTag Nodes at Once:
(make sure you have permission to see the cameras)
## Start a Docker Container and Install the Libraries
* `sudo apt install -y ros-humble-usb-cam`
* `sudo apt install -y ros-humble-isaac-ros-apriltag`
## Edit the Launch and Config Files of `isaac-ros-apriltag`
* `cd /opt/ros/humble/share/isaac_ros_apriltag` (this may not be exactly correct but it should be close)
* Edit the launch file to add namespaces
    1. `sudo apt install nano` (or use vi if you`re weird)
    2. `sudo nano isaac_ros_apriltag_usb_cam.launch.py` (rename this file if you don`t want to keep typing out the long name)
    3. Add your camera name in all of the namespace sections. Ensure that you add a namespace section to the `usb_cam_node` as it doesn`t have the option by default. THIS MUST BE THE SAME NAME AS IS IN THE `aprilTagROSBridge.py`. 
        * **Check `sample launch.py` for an example edited launch file**
    4. Make sure you change the `usb_cam_params_path` to whatever params file you are using for that camera
    5. Copy and paste this launch file for each camera you are going to use, **ensuring that you are changing namespace and params file values**
        1. `sudo cp isaac_ros_apriltag_usb_cam.launch.py isaac_ros_apriltag_usb_camCAMERA2.launch.py` (or some other smart sounding name)
* Edit the config files
    1. Navigate to the config folder
        1. `cd ../`
        2. `cd config`
    2. Edit `usb_cam_params.yaml` to use the camera you want (`sudo nano usb_cam_params.yaml`)
        1. Edit `video_device: "/dev/video0"` to be the camera you want. 
        2. Check what cameras you have using `v4l2-ctl --list-devices`. Each camera gets two, always use the even number. Ex: `/dev/video0`, `/dev/video2`, `/dev/video4`
        3. If you are using a different calibration file, edit `camera_info_url: "package://isaac_ros_apriltag/config/camera_info.yaml"` to be the path to your file (Hint: it`s stored in the same folder as `usb_cam_params.yaml`)
        4. You can edit other properties such as `framerate` here. 
        5. **If you are using the old cameras**: edit `pixel_format` to be `pixel_format: "mjpeg2rgb"` (Use a format that is allowed, it will tell you on startup)
    3. Make sure your camera calibration files are here as well
        1. If you have them in the regular docker container then copy them over from there.
        2. In the docker container (`/workspaces/isaac_ros_dev`): `sudo cp cool_name_for_a_file_here.yaml /opt/ros/humble/share/issac_ros_apriltag/config/another_cool_name` (make sure to replace the names with what you want, **ensuring the same name is put into the corresponding `usb_cam_params.yaml`**)
## 3. Start an AprilTag Node Using the Custom Launch File! (yay!)
* `ros2 launch isaac_ros_apriltag isaac_ros_apriltag_usb_cam.launch.py`
* Replace `isaac_ros_apriltag_usb_cam.launch.py` with the name of your launch file


# Running the `apriltagROSBridge.py`
1. Copy `apriltagROSBridge.py` and `2025-reefscape.json` onto Jetson
    * This could be by using a USB drive, scp from a laptop, git pull, etc. 
    * *You only have to believe that you can succeed, that you can be whatever your heart desires, be willing to work for it, and you can have it. - Oprah Winfrey*
2. Copy the files into the docker container
	1. `mv apriltagROSBridge.py /mnt/nova_ssd/workspaces/isaac_ros_dev/pythonScripts/apriltagROSBridge.py`
	2. I am recalling that location from memory; the start is correct but the middle may not be. You should be able to tab out the location and find it. Good luck!
	3. Make sure you move both files to the same folder
3. Open a docker container
4. Make `apriltagROSBridge.py` executable
	* `chmod +x apriltagROSBridge.py`
5. Run `apriltagROSBridge.py`
	1. Type `./apriltagROSBridge.py` into a docker container
	2. After that fails, install the libraries with `pip install {library}` (pynetworktables, pytransform3d, transformations, etc.)
	3. If it says you are missing `rclpy`, run it in the docker container ._.