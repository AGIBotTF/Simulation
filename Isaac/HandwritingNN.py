import os
# Disable oneDNN optimizations and XLA JIT on CPU.
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'
os.environ['TF_XLA_FLAGS'] = '--tf_xla_cpu_global_jit=false'

import tensorflow as tf
tf.config.optimizer.set_jit(False)

import tensorflow_datasets as tfds
import numpy as np
import cv2
import matplotlib.pyplot as plt

# 1. Load and preprocess the dataset (using EMNIST Balanced as an example)
dataset_name = "emnist/balanced"
(ds_train, ds_test), ds_info = tfds.load(
    dataset_name,
    split=['train', 'test'],
    as_supervised=True,
    with_info=True
)

def preprocess(image, label):
    image = tf.cast(image, tf.float32) / 255.0   # Normalize to [0,1]
    image = tf.image.resize(image, [28, 28])
    image = tf.expand_dims(image, axis=-1)         # Add channel dimension
    # Adjust rotation if necessary (EMNIST images are sometimes rotated)
    image = tf.image.rot90(image, k=1)
    return image, label

ds_train = ds_train.map(preprocess).cache().shuffle(10000).batch(128).prefetch(tf.data.AUTOTUNE)
ds_test  = ds_test.map(preprocess).batch(128).prefetch(tf.data.AUTOTUNE)

# 2. Build a simple CNN model with padding="same" and explicit groups=1
model = tf.keras.models.Sequential([
    tf.keras.layers.Conv2D(
        32, (3, 3),
        activation='relu',
        padding='same',
        groups=1,
        input_shape=(28, 28, 1)
    ),
    tf.keras.layers.MaxPooling2D((2, 2)),
    tf.keras.layers.Conv2D(
        64, (3, 3),
        activation='relu',
        padding='same',
        groups=1
    ),
    tf.keras.layers.MaxPooling2D((2, 2)),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(128, activation='relu'),
    tf.keras.layers.Dense(ds_info.features['label'].num_classes, activation='softmax')
])

model.summary()

model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

# 3. Train the model
model.fit(ds_train, validation_data=ds_test, epochs=10)

# Save the trained model for later use
model.save("emnist_model.h5")

# 4. (Optional) Functions for segmenting and predicting whole words from a drawn canvas image
def segment_characters(image):
    """
    Given a grayscale image (numpy array) of handwritten text,
    segment it into individual character images using contour detection.
    """
    # Invert image so letters are white on black background
    _, thresh = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY_INV)
    # Find contours of the letters
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Create bounding boxes for each contour and sort them left-to-right
    bounding_boxes = sorted([cv2.boundingRect(c) for c in contours], key=lambda b: b[0])
    
    characters = []
    for box in bounding_boxes:
        x, y, w, h = box
        char_img = thresh[y:y+h, x:x+w]
        # Resize each character image to 28x28 to match model input
        char_img = cv2.resize(char_img, (28, 28))
        char_img = char_img.astype('float32') / 255.0
        char_img = np.expand_dims(char_img, axis=-1)  # Add channel dimension
        characters.append(char_img)
    return characters

def predict_word(model, image):
    """
    Given a drawn canvas image (grayscale numpy array) containing a word,
    segment the image and predict each character using the trained model.
    Returns the concatenated string of predictions.
    
    Note: The mapping from model output class to actual character depends on 
    the EMNIST configuration. Here we use a placeholder mapping.
    """
    characters = segment_characters(image)
    word = ""
    # Placeholder mapping for EMNIST balanced: adjust based on your needs.
    mapping = {i: chr(65 + i) for i in range(26)}  # Assuming 0-25 map to A-Z
    for char in characters:
        char_input = np.expand_dims(char, axis=0)  # Add batch dimension
        prediction = model.predict(char_input)
        predicted_class = np.argmax(prediction, axis=1)[0]
        char_pred = mapping.get(predicted_class, '?')
        word += char_pred
    return word

# 5. Example usage:
# Uncomment and adjust these lines to test word prediction.
# canvas_image = cv2.imread("canvas_word.png", cv2.IMREAD_GRAYSCALE)
# if canvas_image is not None:
#     predicted_word = predict_word(model, canvas_image)
#     print("Predicted word:", predicted_word)
# else:
#     print("Canvas image not found. Please check the path.")
