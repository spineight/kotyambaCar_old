# set the matplotlib backend so figures can be saved in the background
import matplotlib
matplotlib.use("Agg")

# import the necessary packages
from nvidia_model import NVIDIA_model
# from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
# from sklearn.metrics import classification_report
# from keras.preprocessing.image import ImageDataGenerator
from keras.optimizers import SGD
from imutils import paths
# import matplotlib.pyplot as plt
import numpy as np
import argparse
import random
import pickle
import cv2
import os

from model_config import *

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--dataset", required=True,
	help="path to input dataset of images")
ap.add_argument("-m", "--model", required=True,
	help="path to output trained model")
# ap.add_argument("-l", "--label-bin", required=True,
# 	help="path to output label binarizer")
# ap.add_argument("-p", "--plot", required=True,
# 	help="path to output accuracy/loss plot")

# I also call vars  on the object to turn the parsed command line arguments 
# into a Python dictionary where the key to the dictionary 
# is the name of the command line argument and the value is value of 
# the dictionary supplied for the command line argument. 
# To see this in action I would suggest inserting a print(args)  
# statement into the code.

# https://www.pyimagesearch.com/2018/03/12/python-argparse-command-line-arguments/
args = vars(ap.parse_args())

# initialize the data and labels
print("[INFO] loading images...")
data = []
labels = []

# grab the image paths and randomly shuffle them
imagePaths = sorted(list(paths.list_images(args["dataset"])))
random.seed(42)
random.shuffle(imagePaths)

# loop over the input images
for imagePath in imagePaths:
    # load the image, resize it to 64x64 pixels (the required input
    # spatial dimensions of SmallVGGNet), and store the image in the
    # data list
    image = cv2.imread(imagePath)
    # One key difference is that we are not flattening our data for neural network,
    # because it is convolutional.
    image = cv2.resize(image, (NVIDIA_W, NVIDIA_H))
    data.append(image)

    # extract the class label from the image path and update the
    # labels list
    label = imagePath.split(os.path.sep)[-1]
    print "imagePath: {}".format(imagePath)
    print "label:{}".format(label)
    print label[:-4].split('_')
    frame_num,linear_velocity, angular_velocity = label[:-4].split('_')
    print "angular_velocity{}".format(angular_velocity)
    labels.append(float(angular_velocity))

# scale the raw pixel intensities to the range [0, 1]
data = np.array(data, dtype="float") / 255.0
labels = np.array(labels)
print "loaded {} frames".format(len(data))

# partition the data into training and testing splits using 75% of
# the data for training and the remaining 25% for testing
(trainX, testX, trainY, testY) = train_test_split(data,
	labels, test_size=0.25, random_state=42)

# convert the labels from integers to vectors (for 2-class, binary
# classification you should use Keras' to_categorical function
# instead as the scikit-learn's LabelBinarizer will not return a
# vector)
# lb = LabelBinarizer()
# trainY = lb.fit_transform(trainY)
# testY = lb.transform(testY)

# construct the image generator for data augmentation
# Image augmentation allows us to construct "additional" training data from our 
# existing training data by randomly rotating, shifting, shearing, zooming, 
# and flipping.
# Data augmentation is often a critical step to:
#   Avoiding overfitting
#   Ensuring your model generalizes well
# aug = ImageDataGenerator(rotation_range=30, width_shift_range=0.1,
# 	height_shift_range=0.1, shear_range=0.2, zoom_range=0.2,
# 	horizontal_flip=True, fill_mode="nearest")

# initialize our VGG-like Convolutional Neural Network
nvidia_net = NVIDIA_model.build(width=NVIDIA_W, height=NVIDIA_H, depth=3)

# initialize our initial learning rate, # of epochs to train for,
# and batch size
INIT_LR = 0.01
EPOCHS = 75
BS = 32

# initialize the model and optimizer (you will want to use
# binary_crossentropy for 2-class classification)
print("[INFO] training network...")
# opt = SGD(lr=INIT_LR, decay=INIT_LR / EPOCHS)
# model.compile(loss="categorical_crossentropy", optimizer=opt,
# 	metrics=["accuracy"])

nvidia_net.compile(optimizer='adam', loss='mse')

# train the network
# Since we are performing data augmentation, we call model.fit_generator  
# (instead of model.fit ). 
# We must pass the generator with our training data as the first parameter. 
# The generator will produce batches of augmented training data according 
# to the settings we previously made.

# H = model.fit_generator(aug.flow(trainX, trainY, batch_size=BS),
# 	validation_data=(testX, testY), steps_per_epoch=len(trainX) // BS,
# 	epochs=EPOCHS)
H = nvidia_net.fit(trainX, trainY, epochs=50, batch_size=128,
    verbose=1)


# evaluate the network
print("[INFO] evaluating network...")
predictions = nvidia_net.predict(testX, batch_size=32)
# print(classification_report(testY.argmax(axis=1),
# 	predictions.argmax(axis=1), target_names=lb.classes_))

# # plot the training loss and accuracy
# N = np.arange(0, EPOCHS)
# plt.style.use("ggplot")
# plt.figure()
# plt.plot(N, H.history["loss"], label="train_loss")
# plt.plot(N, H.history["val_loss"], label="val_loss")
# plt.plot(N, H.history["accuracy"], label="train_accuracy")
# plt.plot(N, H.history["val_accuracy"], label="val_accuracy")
# plt.title("Training Loss and Accuracy (SmallVGGNet)")
# plt.xlabel("Epoch #")
# plt.ylabel("Loss/Accuracy")
# plt.legend()
# plt.savefig(args["plot"])

# # save the model and label binarizer to disk
print("[INFO] serializing network...")
nvidia_net.save(args["model"])
# f = open(args["label_bin"], "wb")
# f.write(pickle.dumps(lb))
# f.close()

# apply trained model:
# python predict.py --image images/panda.jpg --model output/smallvggnet.model \
#	--label-bin output/smallvggnet_lb.pickle --width 64 --height 64