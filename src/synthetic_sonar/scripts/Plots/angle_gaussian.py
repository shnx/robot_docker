import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

# Generate angles from 0 to 15 (rounded to 1)
angles = np.arange(0, 17, 1)

# Calculate probabilities using Gaussian distribution parameters
mean = 8
std_dev = 10
max_probability = 0.95

# Calculate the PDF for each angle
probabilities = norm.pdf(angles, mean, std_dev)

# Normalize probabilities to have a maximum of 0.4
probabilities /= np.max(probabilities)
probabilities *= max_probability

# Round angles to 1 decimal place
rounded_angles = np.round(angles, 1)

# Create a dictionary with rounded angles as keys and corresponding probabilities as values
prob_fa = dict(zip(rounded_angles, probabilities))

# Example: Accessing the probability for a specific rounded angle
rounded_angle = 10.0  # Replace with your actual rounded angle
probability = prob_fa.get(rounded_angle, None)

if probability is not None:
    print(f"The probability for rounded angle {rounded_angle} is: {probability}")
else:
    print(f"No probability found for rounded angle {rounded_angle}")

# Plot the Gaussian distribution
plt.plot(angles, probabilities, marker='o', linestyle='-', color='b')
plt.title('Gaussian Distribution of Probabilities for Angles')
plt.xlabel('Angle')
plt.ylabel('Probability')
plt.show()
