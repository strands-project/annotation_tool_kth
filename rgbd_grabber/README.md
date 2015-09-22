rgbd_grabber
============

This node grabbs images published by the openni ros node (RGB and depth) and saves them on the disk. By default the files are saved in a folder named based on the timestamp generated at the start of the program, however the user can force the creation of new folders where subsequent files will be saved.

Options supported (at runtime, on the console):
- s : save one file.
- a : start saving a sequence of files.
- z : stop saving a sequence of files.
- n : create a new folder.
- i : increase the frame skip by 1 (default is 0, i.e. save all frames).
- u : decrease the frame skip by 1.

The node saves both RGB and depth streams as png files. The naming convention is RGB_SEQ_.png and Depth_SEQ_.png, where _SEQ_ is the sequence number and takes values between 0001 and 9999 (thus a maximum of 10000 images can be saved per folder). In addition, each folder contains an index.txt file where each line contains one file name and the timestamp at which it was saved (taken from the approriapte ros messages). The depth and RGB streams are synchronized based on the timestamps of the original ROS messages.

Note that when in the save sequence mode, if the lens of the device is covered, the node will automatically skip the incomming frames, as well as create a new folder where the new frames will be put, once the lens has been uncovered.

rgbd_grabber
