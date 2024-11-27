# Import required libraries
import tensorflow as tf
from tensorflow.keras.datasets import mnist
from tensorflow.keras.utils import to_categorical

# Step 1: Load the MNIST dataset
# MNIST is a dataset of handwritten digits (0-9)
(x_train, y_train), (x_test, y_test) = mnist.load_data()

# Step 2: Preprocess the data
# Flatten the 28x28 images into a 1D array of 784 features
x_train = x_train.reshape(-1, 28 * 28).astype('float32') / 255  # Normalize pixel values to [0, 1]
x_test = x_test.reshape(-1, 28 * 28).astype('float32') / 255

# One-hot encode the labels (e.g., 3 -> [0, 0, 0, 1, 0, 0, 0, 0, 0, 0])
y_train = to_categorical(y_train, 10)
y_test = to_categorical(y_test, 10)

# Step 3: Define the MLP model
# Use a Sequential model to stack layers
model = tf.keras.Sequential([
    tf.keras.layers.Dense(512, activation='relu', input_shape=(784,)),  # First hidden layer with 512 neurons
    tf.keras.layers.Dense(256, activation='relu'),                      # Second hidden layer with 256 neurons
    tf.keras.layers.Dense(10, activation='softmax')                     # Output layer with 10 neurons (for 10 classes)
])

# Step 4: Compile the model
# Define the optimizer, loss function, and metrics
model.compile(optimizer='adam',                       # Adam optimizer for efficient training
              loss='categorical_crossentropy',        # Loss function for multi-class classification
              metrics=['accuracy'])                   # Metric to monitor model performance

# Step 5: Train the model
# Fit the model to the training data
# Epochs: Number of times the model sees the entire dataset
# Batch size: Number of samples processed before updating weights
model.fit(x_train, y_train, epochs=10, batch_size=128, validation_split=0.2)

# Step 6: Evaluate the model
# Test the model on unseen data and print the accuracy
test_loss, test_acc = model.evaluate(x_test, y_test)
print(f'Test accuracy: {test_acc}')

# (Optional) Step 7: Save the model
# Save the trained model for future use
model.save("mnist_mlp_model.h5")
