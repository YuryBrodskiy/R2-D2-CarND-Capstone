from __future__ import print_function
import tensorflow as tf
sess = tf.Session()
from tensorflow import gfile
from tensorflow.python.framework import graph_util
import os
import shutil
from keras import backend as K
import argparse
from train import deepnn
K.set_session(sess)
K.set_learning_phase(0)

INPUT_TENSOR_NAME = 'tl_classifier_in'
FINAL_TENSOR_NAME = 'tl_classifier_out/Softmax'
FREEZED_PATH = 'tf_files/frozen_classifier.pb'


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--weight_path",
        type=str,
        default='ckpt/some.hdf5',
        help="Path to hdf5 weight file.")
    args = parser.parse_args()

    if os.path.exists('tf_files'):
        shutil.rmtree('tf_files')
    os.makedirs('tf_files')

    model = deepnn()
    model.load_weights(args.weight_path)
    frozen = graph_util.convert_variables_to_constants(
        sess, sess.graph.as_graph_def(), [FINAL_TENSOR_NAME])

    with gfile.FastGFile(FREEZED_PATH, 'wb') as f:
        f.write(frozen.SerializeToString())

    print("Done! Wrote graph to `tf_files`")
