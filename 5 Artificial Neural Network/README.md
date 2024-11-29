# Artificial Neural Network: Perceptron

The **Perceptron** is one of the simplest types of Artificial Neural Networks and serves as the foundation for more complex models. It is a **binary linear classifier** that separates data into two categories using a linear decision boundary.

## Components of the Perceptron

1. **Input Layer**: 
   - Accepts feature vectors $`\mathbf{x} \in \mathbb{R}^n`$, where $`n`$ is the number of features.
2. **Weights**: 
   - A vector $`\mathbf{w}`$ that represents the importance of each feature.
3. **Bias**: 
   - A scalar $`b`$ that allows the model to shift the decision boundary.
4. **Activation Function**:
   - A non-linear function that maps the linear output $`z = \mathbf{w} \cdot \mathbf{x} + b`$ to a binary output $`y \in \{0, 1\}`$.

## Mathematical Formulation

1. **Linear Output**:
   $`z = \mathbf{w} \cdot \mathbf{x} + b`$

2. **Prediction Rule**: The predicted output $`\hat{y}`$ is determined as $`f(z) = \begin{cases} 1 & \text{if } z > 0 \\ 0 & \text{otherwise} \end{cases}`$.

3. **Weight Update Rule**: If the prediction is incorrect, the weights and bias are updated as follows: $`\mathbf{w} = \mathbf{w} + \eta \cdot (y_{\text{true}} - \hat{y}) \cdot \mathbf{x}`$ and $`b = b + \eta \cdot (y_{\text{true}} - \hat{y})`$.
   Here, $`\eta`$ is the learning rate.

## Training Procedure

1. Initialize weights and bias to zero.
2. For each sample:
   - Compute the prediction.
   - Update weights and bias if the prediction is incorrect.
3. Repeat for a fixed number of iterations or until convergence.

## Advantages

- Simple and computationally efficient.
- Works well for linearly separable data.

## Limitations

- Cannot handle non-linear decision boundaries.
- Sensitive to feature scaling.

---

## Example Code

### Step 1: Importing Libraries
We import the necessary libraries for data generation, visualization, and numerical operations.
```python
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn import datasets
```

### Step 2: Generate and Visualize the Dataset
We create a two-class dataset using `sklearn.datasets.make_blobs` and plot it.
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
We split the data into training and testing sets.
```python
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=123)

print("Training samples:", len(X_train))
print("Testing samples:", len(X_test))
```

### Step 4: Define the Activation Function
The Perceptron uses a unit step function for activation.
```python
def unit_step_func(x):
    return np.where(x > 0, 1, 0)
```

### Step 5: Build the Perceptron Class
We define the Perceptron class with methods for initialization, training, and prediction.

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

#### 5.2 - Training (`fit`)
The `fit` method learns weights and bias using the Perceptron update rule.
### Mathematical Representation

The code implements an iterative fitting algorithm for a simple classification task. The process can be represented as follows:

1. **Initialization**:
   - Weights: $`\mathbf{w} = \mathbf{0} \in \mathbb{R}^{n_{\text{features}}}`$
   - Bias: $`b = 0`$

2. **Prediction (Linear Model)**:
   $`\hat{y} = \text{activationFunc}(\mathbf{x} \cdot \mathbf{w} + b)`$

3. **Update Rule (Per Sample)**:
   For each sample $`i`$, calculate:
   - $`\text{update} = \eta (y_i - \hat{y}_i)`$
   - Update weights: $`\mathbf{w} \leftarrow \mathbf{w} + \text{update} \cdot \mathbf{x}_i`$
   - Update bias: $`b \leftarrow b + \text{update}`$

Where:
- $`\eta`$ is the learning rate (`self.lr`).
- $`y_i`$ is the true label (binary, converted to 0 or 1).
- $`\hat{y}_i`$ is the predicted output using the activation function.

This process is repeated for `n_iters` iterations or until convergence.

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

#### 5.3 - Prediction (`predict`)
The `predict` method uses the trained weights and bias for predictions. This method calculates the output of the model for input data $`X`$. The formula is:

1. **Linear Output**:
   $`\text{linearOutput} = X \cdot \mathbf{w} + b`$

2. **Activation Function**:
   $`\hat{y} = \text{activationFunc}(\text{linearOutput})`$

Where:
- $`X`$ is the input matrix of shape $`(n_{\text{samples}}, n_{\text{features}})`$.
- $`\mathbf{w}`$ is the weight vector of shape $`(n_{\text{features}})`$.
- $`b`$ is the bias term.
- $`\hat{y}`$ is the predicted output after applying the activation function.

This method computes the predicted values $`\hat{y}`$ for all samples in $`X`$.

```python
    def predict(self, X):
        linear_output = np.dot(X, self.weights) + self.bias
        return self.activation_func(linear_output)
```

### Step 6: Train the Perceptron
Train the perceptron on the training dataset.
```python
p = Perceptron(learning_rate=0.01, n_iters=1000)
p.fit(X_train, y_train)
```

### Step 7: Evaluate the Perceptron
Calculate the accuracy of the Perceptron model.
```python
def accuracy(y_true, y_pred):
    return np.sum(y_true == y_pred) / len(y_true)

y_pred = p.predict(X_test)
print("Model accuracy:", accuracy(y_test, y_pred))
```

### Step 8: Visualize the Decision Boundary
Visualize the decision boundary learned by the Perceptron.
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
