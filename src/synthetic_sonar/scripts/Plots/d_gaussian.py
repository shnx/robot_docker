import numpy as np
import matplotlib.pyplot as plt

# Generate distances from 0 to 30 meters
distances = np.arange(0.01, 30.00, 0.01)

# Round distances to 3 digits
rounded_distances = np.round(distances, 2)

# Generate probabilities from 0.005 to 0.03
probabilities = np.linspace(0.005, 0.03, len(distances))

# Create a dictionary with rounded distances as keys and corresponding probabilities as values
prob_fa = dict(zip(rounded_distances, probabilities))

# Example: Accessing the probability for a specific rounded distance
rounded_d = 10.12  # Replace with your actual rounded distance
probability = prob_fa.get(rounded_d, None)

if probability is not None:
    print(f"The probability for rounded distance {rounded_d} is: {probability}")
else:
    print(f"No probability found for rounded distance {rounded_d}")

# Visualize the probability distribution
plt.plot(rounded_distances, probabilities, label='Probability Distribution')
plt.title('Probability Distribution for Distances')
plt.xlabel('Rounded Distance (m)')
plt.ylabel('Probability')
plt.legend()
plt.show()
