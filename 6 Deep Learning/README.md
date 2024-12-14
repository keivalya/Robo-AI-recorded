# Multi-Layer Perceptron Learning in TensorFlow

A **Multi-Layer Perceptron (MLP)** is a type of feedforward artificial neural network (ANN) consisting of multiple layers of neurons. Each neuron computes a weighted sum of its inputs $`z = \sum_{i=1}^{n} w_i x_i + b`$ and applies a non-linear activation function $`a = f(z)`$, enabling the network to learn and model complex data patterns.

---

## Key Concepts

### What is a Perceptron?

The basic unit of an MLP is the **perceptron**, which computes the weighted sum $`z = \sum_{i=1}^{n} w_i x_i + b`$, where:
- $`z`$: Weighted sum (input to the activation function),
- $`w_i`$: Weight for each input feature $`x_i`$,
- $`b`$: Bias term (shifts the output).

This sum is passed through an **activation function**, such as $`a = f(z)`$, to introduce non-linearity.

---

### Structure of an MLP

1. **Input Layer**: Receives the raw input features $`x`$ (e.g., pixels for an image).
2. **Hidden Layers**: Applies $`z^{(l)} = W^{(l)} a^{(l-1)} + b^{(l)}`$ and $`a^{(l)} = f(z^{(l)})`$, where:
   - $`l`$: Layer index,
   - $`W^{(l)}`$: Weight matrix for layer $`l`$,
   - $`b^{(l)}`$: Bias vector for layer $`l`$,
   - $`a^{(l-1)}`$: Activations from the previous layer,
   - $`f`$: Non-linear activation function.
3. **Output Layer**: Produces the final predictions, often using the **softmax** function.

---

### Activation Functions Explained

1. **Sigmoid Function**: $`f(z) = \frac{1}{1 + e^{-z}}`$
   - Outputs values between 0 and 1, common for binary classification.

2. **Tanh Function**: $`f(z) = \tanh(z) = \frac{e^z - e^{-z}}{e^z + e^{-z}}`$
   - Outputs values between -1 and 1, useful for centered data.

3. **ReLU (Rectified Linear Unit)**: $`f(z) = \max(0, z)`$
   - Outputs $`z`$ if positive, otherwise 0. Common in hidden layers.

4. **Softmax Function** (for multi-class classification): $`\text{softmax}(z_i) = \frac{e^{z_i}}{\sum_{j} e^{z_j}}`$
   - Converts raw scores into probabilities summing to 1.

---

## Step-by-Step Implementation in TensorFlow

### Step 1: Import Required Libraries and Dataset

```python
import tensorflow as tf
from tensorflow.keras.datasets import mnist
from tensorflow.keras.utils import to_categorical

# Load and preprocess MNIST dataset
(x_train, y_train), (x_test, y_test) = mnist.load_data()
x_train = x_train.reshape(-1, 28 * 28).astype('float32') / 255  # Flatten and normalize
x_test = x_test.reshape(-1, 28 * 28).astype('float32') / 255
y_train = to_categorical(y_train, 10)  # One-hot encode labels
y_test = to_categorical(y_test, 10)
```

---

### Step 2: Define the Model

```python
model = tf.keras.Sequential([
    tf.keras.layers.Dense(512, activation='relu', input_shape=(784,)),  # Hidden layer 1
    tf.keras.layers.Dense(256, activation='relu'),                      # Hidden layer 2
    tf.keras.layers.Dense(10, activation='softmax')                     # Output layer
])
```

---

### Step 3: Compile the Model

```python
model.compile(optimizer='adam',                       # Optimizer
              loss='categorical_crossentropy',        # Loss function
              metrics=['accuracy'])                   # Metrics to monitor
```

---

### Step 4: Train the Model

```python
model.fit(x_train, y_train, epochs=10, batch_size=128, validation_split=0.2)
```

---

### Step 5: Evaluate the Model

```python
test_loss, test_acc = model.evaluate(x_test, y_test)
print(f'Test accuracy: {test_acc}')
```

---

### Visualizing Key Equations in TensorFlow

- **Forward Pass**: $`z^{(l)} = W^{(l)} a^{(l-1)} + b^{(l)}, \quad a^{(l)} = f(z^{(l)})`$.
- **Loss Function (Cross-Entropy)**: $`\text{Loss} = -\sum_{i} y_i \log(\hat{y}_i)`$, where $`y_i`$ is the true label and $`\hat{y}_i`$ is the predicted probability.
- **Backpropagation**: $`W \leftarrow W - \eta \frac{\partial \text{Loss}}{\partial W}`$, where $`\eta`$ is the learning rate.

---

## Conclusion

Implementing a Multi-Layer Perceptron in TensorFlow is modular and straightforward. With this guide, you can:
- Customize the architecture by adjusting layers and neurons.
- Experiment with activation functions like ReLU or softmax.
- Fine-tune the learning process using different hyperparameters.

Start building your own MLP models today!

---
## Bonus
Use the following code to visualize `mnist` dataset in python notebook.
```python
import matplotlib.pyplot as plt
from tensorflow.keras.datasets import mnist

# Load MNIST dataset
(x_train, y_train), (x_test, y_test) = mnist.load_data()

# Visualize some samples from the training set
def visualize_mnist(images, labels, num_samples=16):
    plt.figure(figsize=(10, 10))
    for i in range(num_samples):
        plt.subplot(4, 4, i + 1)
        plt.imshow(images[i], cmap='gray')
        plt.title(f"Label: {labels[i]}")
        plt.axis('off')
    plt.tight_layout()
    plt.show()

# Display 16 random samples
visualize_mnist(x_train, y_train)
```
