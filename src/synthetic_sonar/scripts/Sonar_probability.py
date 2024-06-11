import numpy as np
import matplotlib.pyplot as plt

# Assuming self.gaussian_angles, self.mean_angle, and self.std_deviation are defined

# Calculate unnormalized probabilities
probabilities_unnormalized = np.exp(-(self.gaussian_angles - self.mean_angle)**2 / (2 * self.std_deviation**2)) * 0.9

# Plotting
plt.plot(self.gaussian_angles, probabilities_unnormalized, label='Unnormalized Probabilities')
plt.xlabel('Gaussian Angles')
plt.ylabel('Probabilities')
plt.title('Unnormalized Probabilities Plot')
plt.legend()
plt.show()
