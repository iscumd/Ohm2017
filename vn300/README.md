# VN300 node 
## Features
* Publishes ROS messages for Heading (10 hz), position (8 hz), and status (20 hz) of the GPS
* Asynchronous message publishing

## VectorNav programming library installation
1.	Download VectorNav programming library from Google Drive or from VectorNav CD
1.	In the cpp directory, run the command `make`
1.	Next, run `sudo cp bin/libvnproglib-cpp.a /usr/local/lib/`. This will copy the .a file into the proper location so that the compiler can find it.
1.	Copy the contents of the include folder to `/usr/local/include/vn`. `sudo cp -a --parents include/. /usr/local/include/vn/`
1.	Change directory to the gps package, and copy the libvnproglib-cpp.pc file to `/usr/local/lib/pkgconfig/`. `sudo cp --parents libvnproglib-cpp.pc /usr/local/lib/pkgconfig`.
1.	Build and run the node. 
#### If the node does not compile, verify the permissions for all the installed files, including libvnproglib-cpp.pc
