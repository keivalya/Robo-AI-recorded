import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython.display import HTML
from sklearn.model_selection import train_test_split
from sklearn import datasets

def unit_step_func(x):
    return np.where(x > 0, 1, 0)

class Perceptron:
    def __init__(self, learning_rate=0.01, n_iters=1000):
        self.lr = learning_rate
        self.n_iters = n_iters
        self.activation_func = unit_step_func
        self.weights = None
        self.bias = None
        self.boundary_history = []  # To store decision boundary for visualization

    def fit(self, X, y):
        n_samples, n_features = X.shape
        self.weights = np.zeros(n_features)
        self.bias = 0
        y_ = np.where(y > 0, 1, 0)

        for iteration in range(self.n_iters):
            for idx, x_i in enumerate(X):
                linear_output = np.dot(x_i, self.weights) + self.bias
                y_predicted = self.activation_func(linear_output)
                update = self.lr * (y_[idx] - y_predicted)
                self.weights += update * x_i
                self.bias += update

                # Store every nth frame only for faster animation
                if idx % 10 == 0 and self.weights[1] != 0:  # Avoid division by zero
                    x0_vals = np.array([X[:, 0].min(), X[:, 0].max()])
                    x1_vals = (-self.weights[0] * x0_vals - self.bias) / self.weights[1]
                    self.boundary_history.append((x0_vals, x1_vals))

    def predict(self, X):
        linear_output = np.dot(X, self.weights) + self.bias
        return self.activation_func(linear_output)

def accuracy(y_true, y_pred):
    return np.sum(y_true == y_pred) / len(y_true)

X, y = datasets.make_blobs(
    n_samples=150, n_features=2, centers=2, cluster_std=1.05, random_state=2
)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=123)

p = Perceptron(learning_rate=0.01, n_iters=10)  # Reduce n_iters for faster animation
p.fit(X_train, y_train)
predictions = p.predict(X_test)
print("Perceptron classification accuracy:", accuracy(y_test, predictions))

# Animation setup
fig, ax = plt.subplots()
ax.scatter(X_train[:, 0], X_train[:, 1], c=y_train, cmap="coolwarm", edgecolor="k")
line, = ax.plot([], [], "k-", lw=2)

def init():
    ax.set_xlim(X_train[:, 0].min() - 1, X_train[:, 0].max() + 1)
    ax.set_ylim(X_train[:, 1].min() - 1, X_train[:, 1].max() + 1)
    return line,

def update(frame):
    x_vals, y_vals = p.boundary_history[frame]
    line.set_data(x_vals, y_vals)
    return line,

ani = FuncAnimation(fig, update, frames=range(0, len(p.boundary_history), 5), init_func=init, blit=True)

HTML(ani.to_jshtml())
