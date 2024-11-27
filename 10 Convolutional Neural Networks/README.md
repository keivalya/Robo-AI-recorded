# Convolutional Neural Networks (CNNs)

## Introduction

Convolutional Neural Networks (CNNs) are specialized neural networks designed for processing structured grid data like images. They are the backbone of modern computer vision applications due to their ability to automatically extract and learn features from data.

---

## What are CNNs?

A CNN consists of several layers that transform the input image into progressively higher-level features. The main building blocks of a CNN are:

1. **Convolutional Layers**: Perform convolution operations to detect patterns such as edges or textures.
2. **Pooling Layers**: Reduce spatial dimensions, preserving important features while minimizing computational complexity.
3. **Fully Connected Layers**: Aggregate the features to make predictions.

### Mathematical Representation of Convolution

For an input image $`I(x, y)`$ and a filter $`K(m, n)`$, the convolution operation is defined as:
$`(I * K)(x, y) = \sum_m \sum_n I(x - m, y - n) \cdot K(m, n)`$

---

## Types of Convolutions

### 1. Strided Convolutions
Instead of sliding the filter one pixel at a time, stride skips pixels to reduce the output size.

### 2. Dilated Convolutions
Dilated convolutions introduce gaps (dilations) between filter elements to increase the receptive field without increasing the filter size.

### 3. Padding
Padding adds extra pixels (usually zeros) around the image borders to control the spatial dimensions of the output.

### 4. Pooling Layers
Pooling layers downsample the feature maps to reduce dimensions. Common pooling operations include:

- **Max Pooling**: Takes the maximum value in a pooling window.
- **Average Pooling**: Takes the average of values in a pooling window.

---

## Hands-On Coding with CNNs

### Import Libraries and Prepare Data
```python
import cv2
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense

# Load a sample image
image = cv2.imread("sample.jpg", cv2.IMREAD_GRAYSCALE)  # Replace with your image
plt.imshow(image, cmap="gray")
plt.title("Original Image")
plt.axis("off")
plt.show()

# Reshape image for convolution
image = np.expand_dims(image, axis=(0, -1))  # Batch size 1, height, width, 1 channel
```

### Convolution Operation
```python
# Define a simple convolutional filter
kernel = np.array([[1, 0, -1], [1, 0, -1], [1, 0, -1]], dtype="float32")  # Edge-detection filter
kernel = np.expand_dims(kernel, axis=(-1, -2))  # Shape (3, 3, 1, 1)

# Apply convolution
convolved = cv2.filter2D(image[0, :, :, 0], -1, kernel[:, :, 0, 0])

plt.imshow(convolved, cmap="gray")
plt.title("Convolved Image (Edge Detection)")
plt.axis("off")
plt.show()
```

### CNN Architecture
```python
# Define a CNN
model = Sequential([
    Conv2D(32, (3, 3), activation="relu", input_shape=(image.shape[1], image.shape[2], 1)),
    MaxPooling2D(pool_size=(2, 2)),
    Flatten(),
    Dense(128, activation="relu"),
    Dense(10, activation="softmax")  # Example for 10 classes
])

model.summary()
```

---

## Modern Computer Vision Applications

### 1. YOLOx Series (You Only Look Once)
YOLO is a family of models designed for real-time object detection. YOLOx improves accuracy while maintaining speed. It processes images using a single neural network pass, predicting bounding boxes and class probabilities.

### YOLO Example Code (Using Pretrained YOLO)
```python
# Install dependencies: pip install ultralytics
from ultralytics import YOLO

# Load a pretrained YOLO model
model = YOLO("yolov8n.pt")  # Replace with your YOLO model version

# Perform object detection on an image
results = model("input.jpg", show=True)  # Replace with your image path
```

---

### 2. SAM (Segment Anything Model) by Meta

SAM is a foundation model for image segmentation developed by Meta. It can segment any object in an image with minimal user input.

### Using SAM
```python
# Install Segment Anything: pip install segment-anything
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator

# Load the pretrained SAM model
sam = sam_model_registry["default"](checkpoint="sam_vit_h.pth")  # Path to SAM weights

# Create a mask generator
mask_generator = SamAutomaticMaskGenerator(sam)

# Generate masks for an image
image = cv2.imread("input.jpg")  # Replace with your image path
masks = mask_generator.generate(image)

# Visualize masks
for mask in masks:
    mask_image = mask["segmentation"]
    plt.imshow(mask_image, cmap="jet", alpha=0.5)
plt.title("Segmentation Output")
plt.axis("off")
plt.show()
```

---

## Practical Project: Image Classification with CNNs

```python
# Define a CNN model for MNIST digit classification
from tensorflow.keras.datasets import mnist
from tensorflow.keras.utils import to_categorical

# Load MNIST dataset
(x_train, y_train), (x_test, y_test) = mnist.load_data()
x_train = x_train.reshape(-1, 28, 28, 1) / 255.0  # Normalize and add channel dimension
x_test = x_test.reshape(-1, 28, 28, 1) / 255.0
y_train = to_categorical(y_train, 10)
y_test = to_categorical(y_test, 10)

# Build CNN model
model = Sequential([
    Conv2D(32, (3, 3), activation="relu", input_shape=(28, 28, 1)),
    MaxPooling2D(pool_size=(2, 2)),
    Flatten(),
    Dense(128, activation="relu"),
    Dense(10, activation="softmax")
])

model.compile(optimizer="adam", loss="categorical_crossentropy", metrics=["accuracy"])
model.fit(x_train, y_train, validation_data=(x_test, y_test), epochs=5)

# Evaluate on test data
test_loss, test_accuracy = model.evaluate(x_test, y_test)
print(f"Test Accuracy: {test_accuracy:.2f}")
```

---

## Conclusion

Convolutional Neural Networks (CNNs) are the foundation of modern computer vision. This tutorial covered the basic principles, coding hands-on, and advanced applications like YOLO and SAM. With these skills, you can explore areas like real-time object detection, image segmentation, and image classification.