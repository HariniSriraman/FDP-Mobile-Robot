#!/usr/bin/env python3

import tensorflow as tf
from tensorflow.keras import layers, models
import os


# ==========================================
# SETTINGS
# ==========================================

DATASET_PATH = "dataset"

IMAGE_SIZE = (64, 64)
BATCH_SIZE = 32
EPOCHS = 10

MODEL_NAME = "red_classifier.h5"


# ==========================================
# LOAD DATASET
# ==========================================

print("Loading dataset...")

dataset = tf.keras.utils.image_dataset_from_directory(
    DATASET_PATH,
    image_size=IMAGE_SIZE,
    batch_size=BATCH_SIZE,
    label_mode="binary",
    shuffle=True
)


# ==========================================
# SPLIT DATASET
# ==========================================

dataset_size = tf.data.experimental.cardinality(dataset).numpy()

train_size = int(0.8 * dataset_size)

train_dataset = dataset.take(train_size)
validation_dataset = dataset.skip(train_size)

print("Dataset loaded")
print("Training batches:", train_size)
print("Validation batches:", dataset_size - train_size)


# ==========================================
# IMPROVE DATA PERFORMANCE
# ==========================================

AUTOTUNE = tf.data.AUTOTUNE

train_dataset = train_dataset.prefetch(
    buffer_size=AUTOTUNE
)

validation_dataset = validation_dataset.prefetch(
    buffer_size=AUTOTUNE
)


# ==========================================
# CREATE CNN
# ==========================================

print("Creating CNN model...")

model = models.Sequential([

    # Normalize pixel values
    layers.Rescaling(
        1.0 / 255,
        input_shape=(64, 64, 3)
    ),

    # CNN layer 1
    layers.Conv2D(
        16,
        (3, 3),
        activation="relu"
    ),

    layers.MaxPooling2D(
        (2, 2)
    ),

    # CNN layer 2
    layers.Conv2D(
        32,
        (3, 3),
        activation="relu"
    ),

    layers.MaxPooling2D(
        (2, 2)
    ),

    # CNN layer 3
    layers.Conv2D(
        64,
        (3, 3),
        activation="relu"
    ),

    layers.MaxPooling2D(
        (2, 2)
    ),

    # Convert feature maps to vector
    layers.Flatten(),

    # Fully connected layer
    layers.Dense(
        64,
        activation="relu"
    ),

    # Output
    layers.Dense(
        1,
        activation="sigmoid"
    )
])


# ==========================================
# DISPLAY MODEL
# ==========================================

model.summary()


# ==========================================
# COMPILE MODEL
# ==========================================

model.compile(

    optimizer="adam",

    loss="binary_crossentropy",

    metrics=["accuracy"]
)


# ==========================================
# TRAIN MODEL
# ==========================================

print("\nStarting training...\n")

history = model.fit(

    train_dataset,

    validation_data=validation_dataset,

    epochs=EPOCHS
)


# ==========================================
# SAVE MODEL
# ==========================================

print("\nSaving model...")

model.save(MODEL_NAME)

print("\n================================")
print("Training complete!")
print("Model saved as:", MODEL_NAME)
print("================================")

