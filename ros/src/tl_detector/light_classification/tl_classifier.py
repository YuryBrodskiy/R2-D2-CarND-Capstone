from styx_msgs.msg import TrafficLight
import cv2
import numpy as np
put_text = lambda img, text: cv2.putText(img, text, (10, 50), cv2.FONT_HERSHEY_DUPLEX, 2.0, (255, 127, 127), 3, cv2.LINE_AA) # noqa


class TLClassifier(object):
    def __init__(self):
        # TODO load classifier
        pass

    def get_classification(self, image_bgr):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in
                 styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction
        image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(image_hsv)
        r_thresh = np.logical_and(
            h > 160, s > 100).astype(np.uint8)
        sum_red_pixels = r_thresh.sum()
        debug_image = 255 * np.dstack((r_thresh, r_thresh, r_thresh))
        if sum_red_pixels > 200:
            tl_state = TrafficLight.RED
            put_text(debug_image, "Red: " + str(sum_red_pixels))
        elif sum_red_pixels < 200 and sum_red_pixels > 100:
            tl_state = TrafficLight.YELLOW
            put_text(debug_image, "Yellow: " + str(sum_red_pixels))
        else:
            tl_state = TrafficLight.GREEN
            put_text(debug_image, "Green: " + str(sum_red_pixels))

        return tl_state, debug_image
