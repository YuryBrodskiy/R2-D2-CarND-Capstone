from styx_msgs.msg import TrafficLight
import cv2
import numpy as np
import tensorflow as tf
put_text = lambda img, text: cv2.putText(img, text, (10, 50), cv2.FONT_HERSHEY_DUPLEX, 2.0, (255, 127, 127), 3, cv2.LINE_AA) # noqa


class TLClassifier(object):
    def __init__(self, traffic_light_id=10, min_conf_thresh=0.1,
                 ckpt_path='frozen_inference_graph.pb'):
        self.traffic_light_id = traffic_light_id
        self.min_conf_thresh = min_conf_thresh
        detection_graph = tf.Graph()
        with detection_graph.as_default():
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile(ckpt_path, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

        self.image = detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular
        # object was detected.
        self.boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self.classes = detection_graph.get_tensor_by_name(
            'detection_classes:0')
        self.num_detections = detection_graph.get_tensor_by_name(
            'num_detections:0')
        self.sess = tf.Session(graph=detection_graph)

    def get_classification(self, image_bgr):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in
                 styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        pimg = np.expand_dims(image_rgb, axis=0)
        fetches = [
            self.boxes, self.scores, self.classes, self.num_detections]
        (boxes, scores, classes, num) = self.sess.run(
            fetches,
            feed_dict={self.image: pimg})
        debug_image = image_rgb.copy()
        tl_indics = (classes == self.traffic_light_id)
        tl_boxes = boxes[tl_indics]
        tl_scores = scores[tl_indics]
        for i, tl_box in enumerate(tl_boxes):
            if tl_scores[i] > self.min_conf_thresh:
                h, w = debug_image.shape[:2]
                tl = (int(w * tl_boxes[i, 1]), int(h * tl_boxes[i, 0]))
                br = (int(w * tl_boxes[i, 3]), int(h * tl_boxes[i, 2]))
                cv2.rectangle(
                    debug_image, tl, br, (0, 255, 0), thickness=2)
        put_text(debug_image, 'Red!')
        tl_state = TrafficLight.GREEN
        return tl_state, debug_image
