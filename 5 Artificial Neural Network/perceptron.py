import numpy as np

# Unit step activation function
# Formula: f(x) = 1 if x > 0; otherwise f(x) = 0
def unit_step_func(x):
    return np.where(x > 0, 1, 0)

# Implementation of a Perceptron, a basic building block of an Artificial Neural Network
class Perceptron:

    def __init__(self, learning_rate=0.01, n_iters=1000):
        """
        Initializes the Perceptron with learning rate and number of iterations.

        Parameters:
        - learning_rate (float): The step size for updating weights.
        - n_iters (int): Number of iterations over the training dataset.
        """
        self.lr = learning_rate  # Learning rate
        self.n_iters = n_iters  # Number of iterations for training
        self.activation_func = unit_step_func  # Activation function
        self.weights = None  # Weights of the model
        self.bias = None  # Bias term

    def fit(self, X, y):
        """
        Trains the perceptron on the given dataset.

        Parameters:
        - X (numpy array): Feature matrix of shape (n_samples, n_features)
        - y (numpy array): Labels vector of shape (n_samples,)
        """
        n_samples, n_features = X.shape  # Number of samples and features

        # Initialize weights and bias
        self.weights = np.zeros(n_features)  # Weight vector initialized to zero
        self.bias = 0  # Bias initialized to zero

        # Transform labels into binary format (0 or 1)
        # y_ = 1 if y > 0; otherwise y_ = 0
        y_ = np.where(y > 0, 1, 0)

        # Training loop for n_iters iterations
        for _ in range(self.n_iters):
            for idx, x_i in enumerate(X):
                # Compute the linear output: z = w·x + b
                linear_output = np.dot(x_i, self.weights) + self.bias
                # Apply the activation function: f(z)
                y_predicted = self.activation_func(linear_output)

                # Update rule:
                # w = w + lr * (y_true - y_pred) * x
                # b = b + lr * (y_true - y_pred)
                update = self.lr * (y_[idx] - y_predicted)
                self.weights += update * x_i  # Update weights
                self.bias += update  # Update bias

    def predict(self, X):
        """
        Makes predictions on new data.

        Parameters:
        - X (numpy array): Feature matrix of shape (n_samples, n_features)

        Returns:
        - numpy array: Predicted labels (0 or 1) for each sample.
        """
        # Compute linear output: z = w·x + b
        linear_output = np.dot(X, self.weights) + self.bias
        # Apply the activation function: f(z)
        y_predicted = self.activation_func(linear_output)
        return y_predicted


# Testing the Perceptron
if __name__ == "__main__":
    # Imports for dataset creation and visualization
    import matplotlib.pyplot as plt
    from sklearn.model_selection import train_test_split
    from sklearn import datasets

    # Function to calculate accuracy
    def accuracy(y_true, y_pred):
        """
        Computes accuracy as the ratio of correct predictions to total predictions.
        Formula: accuracy = (Number of Correct Predictions) / (Total Predictions)
        """
        accuracy = np.sum(y_true == y_pred) / len(y_true)
        return accuracy

    # Generate a 2-class synthetic dataset using sklearn
    X, y = datasets.make_blobs(
        n_samples=150, n_features=2, centers=2, cluster_std=1.05, random_state=2
    )

    # Split data into training and test sets
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=123
    )

    # Initialize and train the perceptron
    p = Perceptron(learning_rate=0.01, n_iters=1000)
    p.fit(X_train, y_train)
    predictions = p.predict(X_test)

    # Print the classification accuracy
    print("Perceptron classification accuracy", accuracy(y_test, predictions))

    # Visualization
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    plt.scatter(X_train[:, 0], X_train[:, 1], marker="o", c=y_train)

    # Decision boundary formula: x1 = (-w0*x0 - b) / w1
    x0_1 = np.amin(X_train[:, 0])  # Minimum x-coordinate
    x0_2 = np.amax(X_train[:, 0])  # Maximum x-coordinate

    x1_1 = (-p.weights[0] * x0_1 - p.bias) / p.weights[1]
    x1_2 = (-p.weights[0] * x0_2 - p.bias) / p.weights[1]

    ax.plot([x0_1, x0_2], [x1_1, x1_2], "k")  # Plot decision boundary

    # Adjust plot limits
    ymin = np.amin(X_train[:, 1])
    ymax = np.amax(X_train[:, 1])
    ax.set_ylim([ymin - 3, ymax + 3])

    # Show the plot
    plt.show()
