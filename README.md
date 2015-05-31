
Code to work with Google Project Tango tablet

There are two parts to this project.

The first part is the JavaPointCloud application. This is the demo program
that is included with the Project Tango tablet, but modified to write the
data sets to a directory.

Run the program and press the save button the right side of the screen to
save the point cloud. Each data set includes the pose information, the
point cloud data, and a color image from the camera.

When you have finished scanning, connect the Project Tango tablet to a PC,
and nagivate to the Internal storage/Documents/myScans directory. You should
find one numbered Scan<N>.data and tango<N>.png for each time you hit save. Drag those over to your PC.

The second application is a reader for these data sets. This will read in the
scan file and write out an XYZ file in the same directory. All the points in
the scan file will by transformed by the data from the pose so the data should
all line up.

Run the reader as:
    python reader.py DIR
Where dir is a directory containing the scans.

It will output a few lines of diagnostic information as it processes each of the scans. The output will be a ply file in the same directory.

I've been loading the scan in to meshlab. You should see your scans basically line up.

The reader is really simple, and the matmath.py file includes the details of
the transformations being used to align the data. Hopefully this is a simple
example for others using the tablet.

I don't have a precise roadmap, but my next couple goals are:

1) Modify the reader program to combine all the scans for you. This should be
trivial to do in python.

2) DONE ~~Snap a color photo along with each scan, and use the reader program
to create color point clouds.~~

3) Extend the reader to write a threejs program to display these point clouds
on the web.

4) Use meshlab to learn the best ways to process these point clouds and then
write code to use Point Cloud Library (PCL) to process them wihtout meshlab.

If you're interested, please follow the project, fork the code, provide merge
requests, etc. and we'll see what we can do.

You can reach me as daryll.strauss on gmail.
