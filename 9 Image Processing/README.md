# Image Processing

## Introduction

Image Processing is a fundamental field in computer vision and digital image manipulation. It involves the manipulation and analysis of images to enhance their quality, extract meaningful information, or prepare them for further processing.

Source for Lanna image: [Lanna](http://www.lenna.org/)

### What is an Image?

An image is a visual representation of data. For humans, an image is perceived through vision as a combination of colors and shapes. Computers, on the other hand, interpret an image as a matrix or grid of numerical values representing pixel intensities.

1. **For Humans**:
   - An image is perceived as a combination of colors and patterns.
   - It contains semantic information (e.g., a photo of a dog is understood as "a dog").

2. **For Computers**:
   - An image is a 2D or 3D array of numerical data.
   - In grayscale images, each pixel is represented by a single intensity value (e.g., 0–255 for 8-bit images).
   - In RGB color images, each pixel has three components (Red, Green, Blue), forming a tensor.

Mathematically, an image can be represented as:
$`I(x, y) \rightarrow \text{Intensity at coordinates } (x, y)`$

For RGB:
$`I(x, y) = [R(x, y), G(x, y), B(x, y)]`$

## Image Enhancement Techniques

Image enhancement aims to improve the visual appearance or quality of an image. Common techniques include contrast adjustment, sharpening, and denoising.

### 1. Color Correction

Color correction adjusts the colors of an image to match a desired aesthetic or realistic appearance. Techniques include histogram equalization and white balancing.

#### Histogram Equalization
Histogram equalization redistributes pixel intensity values to span the full range, improving contrast:
$`\text{New Pixel Value} = \frac{\text{Cumulative Distribution}}{\text{Max Cumulative Distribution}} \times 255`$

```python
import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load grayscale image
image = cv2.imread("input.jpg", cv2.IMREAD_GRAYSCALE)

# Histogram equalization
equalized = cv2.equalizeHist(image)

# Display the result
plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1), plt.title("Original"), plt.imshow(image, cmap="gray")
plt.subplot(1, 2, 2), plt.title("Equalized"), plt.imshow(equalized, cmap="gray")
plt.show()
```

---

### 2. Image Sharpening

Image sharpening emphasizes edges and fine details in an image by enhancing high-frequency components. A common method is using a Laplacian filter:
$`\text{Sharpened Image} = \text{Original} + \lambda \cdot \text{Laplacian}`$

```python
# Laplacian filter for sharpening
kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
sharpened = cv2.filter2D(image, -1, kernel)

# Display the result
plt.title("Sharpened Image")
plt.imshow(sharpened, cmap="gray")
plt.show()
```

---

### 3. Edge Detection

Edge detection identifies the boundaries within an image. The Sobel and Canny edge detectors are widely used.

#### Sobel Operator:
$`G_x = \frac{\partial I}{\partial x}, \quad G_y = \frac{\partial I}{\partial y}`$

$`G = \sqrt{G_x^2 + G_y^2}`$

```python
# Sobel Edge Detection
sobelx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=3)
sobely = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=3)
sobel_edge = cv2.magnitude(sobelx, sobely)

plt.title("Sobel Edge Detection")
plt.imshow(sobel_edge, cmap="gray")
plt.show()
```

#### Canny Edge Detection:
$`\text{Edges} = \text{Non-max Suppression}\big(\text{Gradient Magnitude}\big) > \text{Thresholds}`$

```python
# Canny Edge Detection
canny_edges = cv2.Canny(image, 100, 200)

plt.title("Canny Edge Detection")
plt.imshow(canny_edges, cmap="gray")
plt.show()
```

---

### 4. Noise Reduction (Filtering Techniques)

Noise in an image can degrade its quality. Common noise types include Gaussian noise and Salt & Pepper noise. Filters like Gaussian blur and median filters are used for noise reduction.

#### Gaussian Blur:
$`G(x, y) = \frac{1}{2\pi\sigma^2} e^{-\frac{x^2 + y^2}{2\sigma^2}}`$

```python
# Gaussian Blur
blurred = cv2.GaussianBlur(image, (5, 5), 0)

plt.title("Gaussian Blur")
plt.imshow(blurred, cmap="gray")
plt.show()
```

#### Median Filter:
$`\text{Filtered Pixel} = \text{Median of Neighboring Pixels}`$

```python
# Median Filter
median_filtered = cv2.medianBlur(image, 5)

plt.title("Median Filter")
plt.imshow(median_filtered, cmap="gray")
plt.show()
```

---

### 5. Feature Extraction

Feature extraction identifies significant attributes or patterns in an image. Techniques include detecting corners (Harris Corner Detection) and keypoints (SIFT, ORB).

#### Harris Corner Detection:
$`R = \text{det}(M) - k \cdot (\text{trace}(M))^2`$

Where $`M`$ is the gradient covariance matrix.

```python
# Harris Corner Detection
gray = np.float32(image)
dst = cv2.cornerHarris(gray, 2, 3, 0.04)
image[dst > 0.01 * dst.max()] = 255

plt.title("Harris Corners")
plt.imshow(image, cmap="gray")
plt.show()
```

#### SIFT (Scale-Invariant Feature Transform):
Identifies scale and rotation-invariant keypoints.

```python
# SIFT Feature Detection
sift = cv2.SIFT_create()
keypoints, descriptors = sift.detectAndCompute(image, None)
image_sift = cv2.drawKeypoints(image, keypoints, None)

plt.title("SIFT Keypoints")
plt.imshow(image_sift, cmap="gray")
plt.show()
```

---

## Conclusion

Image processing bridges the gap between raw image data and higher-level computer vision tasks. With techniques like color correction, sharpening, edge detection, noise reduction, and feature extraction, we can preprocess images for various applications in AI, medical imaging, autonomous vehicles, and more.