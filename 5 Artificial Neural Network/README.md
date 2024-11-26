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