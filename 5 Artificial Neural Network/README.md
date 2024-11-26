## Artificial Neural Network: Perceptron

The **Perceptron** is one of the simplest types of Artificial Neural Networks and serves as the foundation for more complex models. It is a **binary linear classifier** that separates data into two categories using a linear decision boundary.

### Components of the Perceptron

1. **Input Layer**: 
   - Accepts feature vectors \( \mathbf{x} \in \mathbb{R}^n \), where \( n \) is the number of features.
2. **Weights**: 
   - A vector \( \mathbf{w} \) that represents the importance of each feature.
3. **Bias**: 
   - A scalar \( b \) that allows the model to shift the decision boundary.
4. **Activation Function**:
   - A non-linear function that maps the linear output \( z = \mathbf{w} \cdot \mathbf{x} + b \) to a binary output \( y \in \{0, 1\} \).

### Mathematical Formulation

1. **Linear Output**:
   \[
   z = \mathbf{w} \cdot \mathbf{x} + b
   \]

2. **Prediction Rule**:
   \[
   \hat{y} = f(z) = 
   \begin{cases} 
   1 & \text{if } z > 0 \\ 
   0 & \text{otherwise} 
   \end{cases}
   \]

3. **Weight Update Rule**:
   - If the prediction is incorrect:
   \[
   \mathbf{w} = \mathbf{w} + \eta \cdot (y_{\text{true}} - \hat{y}) \cdot \mathbf{x}
   \]
   \[
   b = b + \eta \cdot (y_{\text{true}} - \hat{y})
   \]

   Here, \( \eta \) is the learning rate.

### Training Procedure

1. Initialize weights and bias to zero.
2. For each sample:
   - Compute the prediction.
   - Update weights and bias if the prediction is incorrect.
3. Repeat for a fixed number of iterations or until convergence.

### Advantages

- Simple and computationally efficient.
- Works well for linearly separable data.

### Limitations

- Cannot handle non-linear decision boundaries.
- Sensitive to feature scaling.

### Example Output

- Visualization of decision boundary separating two classes in a 2D dataset.
- Classification accuracy metric for model evaluation.

## Step-by-step code

### Step 1: Importing libraries
First, we import all the libraries needed for data generation, visualization, and numerical operations.
```python
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn import datasets
```

### Step 2: Generate and Visualize the Dataset
We create a simple two-class dataset using `sklearn.datasets.make_blobs`. Then, we plot the dataset to understand its structure.
```python
X, y = datasets.make_blobs(n_samples=150, n_features=2, centers=2, cluster_std=1.05, random_state=2)

plt.figure(figsize=(8, 6))
plt.scatter(X[:, 0], X[:, 1], c=y, cmap='viridis', s=50)
plt.title("Generated Dataset")
plt.xlabel("Feature 1")
plt.ylabel("Feature 2")
plt.show()
```

### Step 3: Split the Dataset
```python
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=123)

print("Training samples:", len(X_train))
print("Testing samples:", len(X_test))
```

### Step 4: Define the Activation Function
```python
def unit_step_func(x):
    return np.where(x > 0, 1, 0)
```

### Step 5: Build the Perceptron Class
We define the Perceptron class step-by-step, explaining its initialization, training (`fit`), and prediction (`predict`) methods.
#### 5.1 - Initialization
```python
class Perceptron:
    def __init__(self, learning_rate=0.01, n_iters=1000):
        self.lr = learning_rate
        self.n_iters = n_iters
        self.activation_func = unit_step_func
        self.weights = None
        self.bias = None
```

#### 5.2 Training (`fit`)
The `fit` method learns weights and bias using the perceptron update rule.
```python
    def fit(self, X, y):
        n_samples, n_features = X.shape
        self.weights = np.zeros(n_features)
        self.bias = 0
        y_ = np.where(y > 0, 1, 0)

        for _ in range(self.n_iters):
            for idx, x_i in enumerate(X):
                linear_output = np.dot(x_i, self.weights) + self.bias
                y_predicted = self.activation_func(linear_output)

                update = self.lr * (y_[idx] - y_predicted)
                self.weights += update * x_i
                self.bias += update
```

#### 5.3 Prediction (`predict`)
The `predict` method uses the trained weights and bias to make predictions on new data.
```python
    def predict(self, X):
        linear_output = np.dot(X, self.weights) + self.bias
        return self.activation_func(linear_output)
```

### Step 6: Train the Perceptron
```python
p = Perceptron(learning_rate=0.01, n_iters=1000)
p.fit(X_train, y_train)
```

### Step 7: Evaluate the Perceptron
```python
def accuracy(y_true, y_pred):
    return np.sum(y_true == y_pred) / len(y_true)
y_pred = p.predict(X_test)
print("Model accuracy:", accuracy(y_test, y_pred))
```

### Step 8: Visualize the Decision Boundary
Finally, we visualize the decision boundary learned by the perceptron.
```python
plt.figure(figsize=(8, 6))
plt.scatter(X_train[:, 0], X_train[:, 1], c=y_train, cmap='viridis', s=50)

x0_1 = np.amin(X_train[:, 0])
x0_2 = np.amax(X_train[:, 0])
x1_1 = (-p.weights[0] * x0_1 - p.bias) / p.weights[1]
x1_2 = (-p.weights[0] * x0_2 - p.bias) / p.weights[1]

plt.plot([x0_1, x0_2], [x1_1, x1_2], 'k', label="Decision Boundary")
plt.title("Decision Boundary")
plt.xlabel("Feature 1")
plt.ylabel("Feature 2")
plt.legend()
plt.show()
```