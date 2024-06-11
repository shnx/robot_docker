import numpy as np

np.random.seed(2)
# Example matrix A (6x6)
A = np.random.randint(low=0,high=4,size=(6, 6))
#A=np.random.randint

# Desired size for the scaled-down matrix (e.g., 4x4 or 3x3)
target_size = (4, 4)

# Calculate the row and column indices to keep
row_indices = np.linspace(0,5,3, dtype=int)
col_indices = np.linspace(0,5,3, dtype=int)
print(row_indices)
# Select the subset of rows and columns
scaled_down_A = A[row_indices][:, col_indices]

print("Original Matrix A:")
print(A)
print("\nScaled-Down Matrix:")
print(scaled_down_A)
