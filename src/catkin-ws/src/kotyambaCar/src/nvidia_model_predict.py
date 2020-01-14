# import the necessary packages
from keras.models import load_model
import argparse
# import pickle
import cv2
from model_config import *

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
	help="path to input image we are going to classify")
ap.add_argument("-m", "--model", required=True,
	help="path to trained Keras model")
# ap.add_argument("-l", "--label-bin", required=True,
# 	help="path to label binarizer")
# ap.add_argument("-w", "--width", type=int, default=28,
# 	help="target spatial dimension width. for our CNN")
# ap.add_argument("-e", "--height", type=int, default=28,
# 	help="target spatial dimension height. for our CNN")
# ap.add_argument("-f", "--flatten", type=int, default=-1,
# 	help="whether or not we should flatten the image. If you need to flatten the image, you should pass a 1  for this argument")
args = vars(ap.parse_args())

# load the input image and resize it to the target spatial dimensions
image = cv2.imread(args["image"])
output = image.copy()
image = cv2.resize(image, (NVIDIA_W, NVIDIA_H))

# scale the pixel values to [0, 1]
image = image.astype("float") / 255.0

# otherwise, we must be working with a CNN -- don't flatten the
# image, simply add the batch dimension
image = image.reshape((1, image.shape[0], image.shape[1],
    image.shape[2]))

# load the model
print("[INFO] loading network...")
model = load_model(args["model"])
# lb = pickle.loads(open(args["label_bin"], "rb").read())

# make a prediction on the image
preds = model.predict(image)

# find the class label index with the largest corresponding
# probability
# i = preds.argmax(axis=1)[0]
# label = lb.classes_[i]

# draw the class label + probability on the output image
text = "{:.2f}%".format(float(preds))
cv2.putText(output, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
	(0, 0, 255), 2)

# show the output image
cv2.imshow("Image", output)
cv2.waitKey(0)

# running:
# python predict.py --image images/cat.jpg --model output/simple_nn.model \
#	--label-bin output/simple_nn_lb.pickle --width 32 --height 32 --flatten 1