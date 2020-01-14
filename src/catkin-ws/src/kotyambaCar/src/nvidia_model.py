from keras.models import Model
from keras.layers import Input, Convolution2D, Flatten, Dense, Dropout, ELU, Lambda
from keras.callbacks import ModelCheckpoint, CSVLogger
import keras.backend as K
from model_config import *
# from load_data import generate_data_batch, split_train_val
from sklearn.model_selection import train_test_split

class NVIDIA_model:
    @staticmethod
    def build(width, height, depth):
        # initialize the model along with the input shape to be
        # "channels last" and the channels dimension itself
        # depth  can also be thought of as the number of channels. 
        # Our images are in the RGB color space, so we will pass a depth  of 3  
        # when we call the build  method
        init = 'glorot_uniform'

        input_frame = Input(shape=(height, width, depth))

        # if we are using "channels first", update the input shape
        # and channels dimension
        if K.image_data_format() == "channels_first":
            input_frame = Input(shape=(depth, height, width))
            chanDim = 1
        # standardize input
        x = Lambda(lambda z: z / 127.5 - 1.)(input_frame)

        x = Convolution2D(24, 5, 5, border_mode='valid', subsample=(2, 2), init=init)(x)
        x = ELU()(x)
        x = Dropout(0.2)(x)
        x = Convolution2D(36, 5, 5, border_mode='valid', subsample=(2, 2), init=init)(x)
        x = ELU()(x)
        x = Dropout(0.2)(x)
        x = Convolution2D(48, 5, 5, border_mode='valid', subsample=(2, 2), init=init)(x)
        x = ELU()(x)
        x = Dropout(0.2)(x)
        x = Convolution2D(64, 3, 3, border_mode='valid', init=init)(x)
        x = ELU()(x)
        x = Dropout(0.2)(x)
        x = Convolution2D(64, 3, 3, border_mode='valid', init=init)(x)
        x = ELU()(x)
        x = Dropout(0.2)(x)

        x = Flatten()(x)

        x = Dense(100, init=init)(x)
        x = ELU()(x)
        x = Dropout(0.5)(x)
        x = Dense(50, init=init)(x)
        x = ELU()(x)
        x = Dropout(0.5)(x)
        x = Dense(10, init=init)(x)
        x = ELU()(x)
        out = Dense(1, init=init)(x)

        model = Model(input=input_frame, output=out)

        model.summary()
        return model