import numpy as np
import matplotlib.pyplot as plt
import numpy as np

# Set seed for reproducibility
np.random.seed(42)

# Function to generate dummy data for grayscale image
def generate_dummy_data(size):
    return np.random.randint(0, 256, size*size)

# Set the size for the images
size1 = 8  # Adjust the size as needed
size2 = 4  # Adjust the size as needed

# Generate dummy data for data1 and data2
data1 = generate_dummy_data(size1)
data2 = generate_dummy_data(size2)

# Print the generated data for verification
print("Data1:", data1)
print("Data2:", data2)


# Assuming data1 and data2 are lists containing pixel values for grayscale images

# Function to reshape and normalize the image
def preprocess_image(data, size):
    image = np.array(data).reshape(size, size)
    # Normalize pixel values to be between 0 and 1
    image = image / 255.0
    return image

# Set the size for the images
size1 = int(np.sqrt(len(data1)))
size2 = int(np.sqrt(len(data2)))

# Reshape and preprocess the images
image1 = preprocess_image(data1, size1)
image2 = preprocess_image(data2, size2)

# Create a figure with two subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

# Plot the first image on the left subplot
ax1.imshow(image1, cmap='gray')
ax1.set_title('Image 1')

# Plot the second image on the right subplot
ax2.imshow(image2, cmap='gray')
ax2.set_title('Image 2')

# Adjust spacing between subplots
plt.tight_layout()

# Create a new figure for the overlap image
fig_overlap, ax_overlap = plt.subplots()

# Calculate the overlapping region
overlap_size = min(size1, size2)
overlap_image = np.zeros((overlap_size, overlap_size))

# Calculate the center offset for both images
offset1 = (size1 - overlap_size) // 2
offset2 = (size2 - overlap_size) // 2

# Place the overlapped region from the first image
overlap_image[:size1, :size1] = image1[offset1:offset1 + overlap_size, offset1:offset1 + overlap_size]

# Place the overlapped region from the second image
overlap_image[:size2, :size2] += image2[offset2:offset2 + overlap_size, offset2:offset2 + overlap_size]

# Normalize the overlapped region to be between 0 and 1
overlap_image = np.clip(overlap_image, 0, 1)

# Plot the overlapped image
ax_overlap.imshow(overlap_image, cmap='gray')
ax_overlap.set_title('Overlap Image')

# Show the figures
plt.show()
