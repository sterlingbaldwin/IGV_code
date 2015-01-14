  CSUchico Intelligent Ground Vehicle Computer Systems summery.
  Sterling Baldwin
  Last revised 1/14/15

-----hardware-----
Motherboard: Intel i5, 4gb ram. Nothing fancy, we were hoping to get some one-time
                    money for an i7 but it doesnt look like thats going to come through.

Raspberry pie: Its a B+ model.

GPS: Adafruit ultimate GPS breakout. see http://www.adafruit.com/product/746

Compass/accelerometer: Adafruit Triple-axis Accelerometer+Magnetometer (Compass) Board - LSM303
                        see http://www.adafruit.com/products/1120#tutorials

Camera: PointGrey Bumblebee 2 see http://www.ptgrey.com/tan/10548 God damn I wish
          I had seen that link like a year ago and saved myself so much time . . .

Motor Control: Galil something something I dont have the model infront of me. I think its this http://www.galil.com/download/datasheet/ds_41x3.pdf
                see http://www.galil.com/download/comref/com41x3/doc.pdf


----Software-----

The main computer is running Xubuntu 14.04, and uses the following
software systems to communicate and interpret sensor data.

            Flycapture: Used with the BB2 camera to communicate with and captura data.
                        see http://www.ptgrey.com/flycapture-sdk

            Triclops: The other side of flycapture, that turns the images taken from
                      the camera and turns them into 3d points.
                      see http://www.ptgrey.com/triclops

            Point Cloud Library: Used to hold and manipulate the point cloud objects.
                                    Extremely useful and well documented, all the data manipulation
                                    and processing goes on in here.
                                    see http://pointclouds.org/documentation/tutorials/

                                  An example of all three of these technologies to take and then display
                                    a point cloud image from the camera can be found here: https://github.com/sterlingbaldwin/IGV_code/blob/master/bb2cloud/bb2viewer.cpp

                                  Get image data -> Create empty visual field grid model -> Analyse the image to get obstacle locations
                                    and place non-passable entries in the model. Once all non-passable locations have been identified,
                                    find the current target location in the grid -> Generate path to target -> Execute path

                                  My main plan here is to grab the cloud image from the Triclops and put it directly into
                                    a pcl cloud. From there we need to find two things: 1) the white lines denoting walls,
                                    and 2) barrels/flags/cone obstacles to avoid and/or direct our movement within the allowed
                                    space. To find the lines, first do a planar segmentation pass to remove all non-ground
                                    points from the image, then do a pass of luminosity filtering removing all points but the
                                    top 10% of whiteness points. Then we use the lines to mark their corresponding location
                                    in the field model as non-passable.

                                  From the planar segmentation, we take the outliers, ie. the points that dont belong to the
                                      ground (and then must be cones/barrels/flags) check if their flags, and otherwise also
                                      place their corresponding points to the non-passable set.

                                  Once we have the location of all the obstacles accounted for, we mark the grid location
                                      that is both the closest to the destination, but also adjacent to unexplored locations.
                                      When we have our short term target, then we can create a minimum-spanning tree and
                                      follow its branch to the target node.


            Galil: The controller is set to a static IP address of 192.168.1.3. The controller can be
                    communicated directly over its socket interface, which we do with a simple python
                    script. See https://github.com/sterlingbaldwin/IGV_code/blob/master/utils/girshell.py
