import numpy as np
import matplotlib.pyplot as plt
import math

# Function representing the brachistochrone curve decay
def decay(ratio, maxSamples, minSamples):
    #Sigmoid function to smooth ratio
    smoothed_value = 1 / (1 + math.exp(-10 * (ratio - 0.5)))
    theta = math.pi / 2 * smoothed_value
    brachistochrone_factor = math.sqrt(math.sin(theta))
    return minSamples +(maxSamples - minSamples) * brachistochrone_factor
    # return maxSamples -(maxSamples - minSamples) * brachistochrone_factor
    # return smoothed_value

# Generate ratio values from 0 to 1
ratios = np.linspace(0, 1, 400)

# Assuming maxSamples and minSamples values for demonstration
maxSamples = 199
minSamples = 1

# Compute decay values for each ratio
values = [decay(r, maxSamples, minSamples) for r in ratios]

# Plotting the decay curve
plt.plot(ratios, values, label="Brachistochrone Decay", color='blue')
plt.xlabel("Ratio")
plt.ylabel("Value")
plt.title("Brachistochrone Curve Decay")
plt.legend()
plt.grid(True)
plt.show()
