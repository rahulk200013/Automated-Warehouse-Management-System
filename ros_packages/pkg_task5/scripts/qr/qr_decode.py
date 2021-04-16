#!/usr/bin/env python

"""
This node handles the job to do the image processing of the image captured by 2D Camera
to identify the colour of packages on the shelf.

The image is cropped into 12 images, each image has one of the package in it. Then
QR code in these images are decoded to identify the colour of the packages at each row
and column of the shelf.
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pyzbar.pyzbar import decode

from pkg_task5.msg import Camera1Image


class ShelfCamera(object):
    """
    Class to analyze 2D camera image and store the information regarding
    colour of the packages
    """
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)
        self.loop_count = 0
        self.pkg_colour_list = ['NA' for _index in range(12)]
        self.all_colour_detected = False

    def callback(self, data):
        """
        This is a callback function called when the 2D camera publishes its feed on the
        ROS Topic "/eyrc/vb/camera_1/image_raw"
        :param data: feed from the 2D camera
        :return: None
        """
        if self.loop_count == 0:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

                # Iterate through each row and column of shelf
                for row in range(4):
                    for column in range(3):
                        # Start and final horizontal coordinates of the respective package
                        x_start = 280 + row * 165
                        x_final = x_start + 165

                        # Start and final vertical coordinates of the respective package
                        y_start = 80 + column * 190
                        y_final = y_start + 190

                        # Image of the respective row and column
                        cropped_image = cv_image[x_start:x_final, y_start:y_final]

                        # Store the decode colour of the package in a list
                        self.pkg_colour_list[3 * row + column] = decode_qr(cropped_image)

                # Number of times the image processing is done
                self.loop_count = 1

                self.all_colour_detected = True

                del self.image_sub

            except CvBridgeError as exception:
                rospy.logerr(exception)

    def get_pkg_colour_list(self):
        """Returns the list of colours of the package."""
        return self.pkg_colour_list


def detect_colours():
    """
    Detects the colours of the packages present on the shelf and publishes them on a ROS Topic.
    :return: None
    """

    # Initialize object of class Camera1 (2D Camera)
    shelf_camera = ShelfCamera()

    # Topic and type of msg to publish the detected colour
    pkg_colour_pub = rospy.Publisher('eyrc/package/colours', Camera1Image, queue_size=10)

    # Wait for all the colour of packages to be detected
    while shelf_camera.all_colour_detected is not True:
        continue

    # Publish the detected colours of all the packages
    pkg_colour_pub.publish(shelf_camera.pkg_colour_list)


def decode_qr(arg_image):
    """
    Decodes the QR code present in the image
    :param arg_image: image in which QR code is present
    :return: decoded data
    """
    qr_result = decode(arg_image)

    if bool(qr_result):
        decoded_data = qr_result[0].data
    else:
        decoded_data = "NA"

    # Return the decoded data from QR code.
    return decoded_data
