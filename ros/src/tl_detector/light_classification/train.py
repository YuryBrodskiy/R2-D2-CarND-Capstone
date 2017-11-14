from glob import glob
import keras
from keras.layers import *
from keras.models import Model
from sklearn.model_selection import train_test_split
from skimage.transform import rotate, warp, resize
from IPython import embed
import cv2
import os
import shutil

INPUT_TENSOR_NAME = 'tl_classifier_in'
FINAL_TENSOR_NAME = 'tl_classifier_out'


def fns_labels():
    train_fns = sorted(glob('data/*/*/*.png'))
    y_labels = []
    for fn in train_fns:
        if 'NoTrafficLight' in fn:
            y_labels.append(0)
        elif 'Green' in fn:
            y_labels.append(1)
        elif 'Yellow' in fn:
            y_labels.append(2)
        elif 'Red' in fn:
            y_labels.append(3)
    return train_fns, y_labels


def image_label_gen(image_fns, labels, batch_size=32, augmentation_fn=None):
    num_images = (len(image_fns) // batch_size) * batch_size
    perm = np.arange(num_images)
    X = np.zeros((batch_size, 64, 32, 3), dtype=np.float32)
    y = np.zeros(batch_size, dtype=np.float32)
    while True:
        np.random.shuffle(perm)
        batch_counter = 0
        for i in range(num_images):
            img = cv2.imread(image_fns[perm[i]])[:, :, ::-1]
            img = cv2.resize(img, (32, 64))
            if augmentation_fn is not None:
                img = augmentation_fn(img)
            X[batch_counter, :] = img
            y[batch_counter] = labels[perm[i]]
            batch_counter += 1
            if batch_counter == batch_size:
                batch_counter = 0
                yield X, y


def deepnn():
    input_layer = Input(shape=[64, 32, 3], name=INPUT_TENSOR_NAME)
    x = input_layer
    x = BatchNormalization()(x)
    x = Conv2D(16, 3, activation='relu')(x)
    x = MaxPool2D()(x)
    x = Conv2D(32, 3, activation='relu')(x)
    x = MaxPool2D()(x)
    x = Conv2D(64, 3, activation='relu')(x)
    x = GlobalAveragePooling2D()(x)
    x = Dense(32, activation='relu')(x)
    x = Dense(4, activation='softmax', name=FINAL_TENSOR_NAME)(x)
    return Model(input_layer, x)


def random_shift(xy):
    xy[:, 0] += np.random.uniform(-10,10)
    xy[:, 1] += np.random.uniform(-20,20)
    return xy

def augment_image(img):
    if np.random.rand() > 0.5:
        img = np.fliplr(img)
    if np.random.rand() > 0.5:
        img = rotate(img, angle=np.random.uniform(-15,15))
    if np.random.rand() > 0.5:
        img = warp(img, random_shift)
    return img


if __name__ == '__main__':
    if os.path.exists('ckpt'):
        shutil.rmtree('ckpt')
    os.makedirs('ckpt')
    num_classes = 4
    batch_size = 32
    epochs = 20
    lr = 0.001
    train_fns, y_labels = fns_labels()
    train_fns, val_fns, y_train, y_val = train_test_split(
        train_fns, y_labels, test_size=0.2)

    train_gen = image_label_gen(train_fns, y_train, batch_size=batch_size,
                                augmentation_fn=augment_image)
    val_gen = image_label_gen(val_fns, y_val, batch_size=batch_size)

    model = deepnn()
    model.compile(
        loss=keras.losses.sparse_categorical_crossentropy,
        optimizer=keras.optimizers.Adam(lr=lr),
        metrics=[keras.metrics.sparse_categorical_accuracy])

    model.fit_generator(
        train_gen,
        steps_per_epoch=len(train_fns) // batch_size,
        validation_data=val_gen,
        validation_steps=len(val_fns) // batch_size,
        epochs=epochs,
        verbose=1,
        callbacks=[keras.callbacks.ModelCheckpoint(
            'ckpt/model-{epoch:03d}-{val_loss:.4f}.hdf5',
            monitor='val_loss')])
