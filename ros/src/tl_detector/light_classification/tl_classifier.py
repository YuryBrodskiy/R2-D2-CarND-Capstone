from styx_msgs.msg import TrafficLight
import cv2
import numpy as np
import tensorflow as tf
import rospy
put_text = lambda img, text, center, col: cv2.putText(img, text, center, cv2.FONT_HERSHEY_DUPLEX, 2.0, col, 3, cv2.LINE_AA) # noqa


class TLClassifier(object):
    def __init__(self, traffic_light_id=10, min_conf_thresh=0.08,
                 detection_ckpt_path='frozen_inference_graph.pb',
                 classifier_ckpt_path='frozen_classifier.pb',
                 reduce_computation=True,
                 gpu_memory_fraction=0.8):
        self.traffic_light_id = traffic_light_id
        self.min_conf_thresh = min_conf_thresh
        self.reduce_computation = reduce_computation
        detection_graph = tf.Graph()
        # load detection graph
        with detection_graph.as_default():
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile(detection_ckpt_path, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

        # load classification graph
        with detection_graph.as_default():
            classifier_graph_def = tf.GraphDef()
            with tf.gfile.GFile(classifier_ckpt_path, 'rb') as fid:
                serialized_graph = fid.read()
                classifier_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(classifier_graph_def, name='')

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
        self.classifier_in = detection_graph.get_tensor_by_name(
            'tl_classifier_in:0')
        self.classifier_out = detection_graph.get_tensor_by_name(
            'tl_classifier_out/Softmax:0')
        # https://github.com/tensorflow/tensorflow/issues/5354
        gpu_options = tf.GPUOptions(
            per_process_gpu_memory_fraction=gpu_memory_fraction)
        config = tf.ConfigProto(
            gpu_options=gpu_options, log_device_placement=False)
        self.sess = tf.Session(config=config, graph=detection_graph)

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
        tic = rospy.get_time()
        (boxes, scores, classes, num) = self.sess.run(
            fetches,
            feed_dict={self.image: pimg})
        toc = rospy.get_time()
        rospy.logwarn("Detection took %f secs" % (toc - tic))
        debug_image = image_rgb.copy()
        tl_indics = (classes == self.traffic_light_id)
        tl_boxes = boxes[tl_indics]
        tl_scores = scores[tl_indics]
        if self.reduce_computation:
            # use only the higest scoring box
            if len(tl_scores) != 0:
                highest_score = np.argmax(scores)
                tl_boxes = tl_boxes[highest_score: highest_score + 1]
                tl_scores = [tl_scores[highest_score]]

        tl_state = TrafficLight.UNKNOWN
        tl_text = 'Unknown'
        tl_color = (0, 0, 0)
        for i, tl_box in enumerate(tl_boxes):
            if tl_scores[i] > self.min_conf_thresh:
                h, w = debug_image.shape[:2]
                tl = (int(w * tl_boxes[i, 1]), int(h * tl_boxes[i, 0]))
                br = (int(w * tl_boxes[i, 3]), int(h * tl_boxes[i, 2]))
                area = (br[1] - tl[1]) * (br[0] - tl[0])
                if area < 12:
                    continue

                patch = cv2.resize(
                    image_rgb[tl[1]: br[1], tl[0]: br[0]], (32, 64))
                patch = np.expand_dims(patch, axis=0)
                predicted_prob = self.sess.run(
                    self.classifier_out,
                    feed_dict={self.classifier_in: patch}).squeeze()
                predicted_light = predicted_prob.argmax(axis=-1)
                max_prob = predicted_prob[predicted_light]
                if predicted_light == 0:
                    tl_state = TrafficLight.UNKNOWN
                    tl_text = 'Unknown: %.2f' % max_prob
                    tl_color = (255, 255, 255)
                elif predicted_light == 1:
                    tl_state = TrafficLight.GREEN
                    tl_text = 'Green: %.2f' % max_prob
                    tl_color = (0, 255, 0)
                elif predicted_light == 2:
                    tl_state = TrafficLight.YELLOW
                    tl_text = 'Yellow: %.2f' % max_prob
                    tl_color = (255, 255, 0)
                elif predicted_light == 3:
                    tl_state = TrafficLight.RED
                    tl_text = 'Red: %.2f' % max_prob
                    tl_color = (255, 0, 0)

                tl_center = (int(0.5 * (br[0] + tl[0])),
                             int(0.5 * (br[1] + tl[1])))
                put_text(debug_image, tl_text, tl_center, tl_color)
                cv2.rectangle(
                    debug_image, tl, br, tl_color, thickness=2)
        return tl_state, debug_image
