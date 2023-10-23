import numpy as np
import matplotlib.pyplot as plt
import math

# Set the default font
# plt.rcParams["font.family"] = "Times New Roman"
# fig, ax = plt.subplots()
# Function representing the logarithmic decay
def decay(ratio, lambda_, maxSamples, minSamples):
    smoothed_value = 1 / (1 + math.exp(-10 * (ratio - 0.5)))
    decay_factor =  np.log(1 + lambda_ * smoothed_value) / np.log(1 + lambda_)
    return minSamples + (maxSamples - minSamples) * decay_factor
# Parameters
minSamples = 1
maxSamples = 199
lambda_ = (minSamples+maxSamples)/4
# Generate ratio values
ratios = np.linspace(0, 1, 500)
batch_sizes = [decay(r, lambda_, maxSamples, minSamples) for r in ratios]
batch_sizes1 = [decay(r, 10, maxSamples, minSamples) for r in ratios]
batch_sizes2 = [decay(r, 20, maxSamples, minSamples) for r in ratios]
batch_sizes3 = [decay(r, 30, maxSamples, minSamples) for r in ratios]
batch_sizes4 = [decay(r, 40, maxSamples, minSamples) for r in ratios]
batch_sizes6 = [decay(r, 60, maxSamples, minSamples) for r in ratios]
batch_sizes7 = [decay(r, 70, maxSamples, minSamples) for r in ratios]
batch_sizes8 = [decay(r, 80, maxSamples, minSamples) for r in ratios]
# Plotting
plt.plot(ratios, batch_sizes1, label=f"Lambda={10}")
plt.plot(ratios, batch_sizes2, label=f"Lambda={20}")
plt.plot(ratios, batch_sizes3, label=f"Lambda={30}")
plt.plot(ratios, batch_sizes4, label=f"Lambda={40}")
plt.plot(ratios, batch_sizes, label=f"Lambda={50}")
plt.plot(ratios, batch_sizes6, label=f"Lambda={60}")
plt.plot(ratios, batch_sizes7, label=f"Lambda={70}")
plt.plot(ratios, batch_sizes8, label=f"Lambda={80}")
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)
plt.legend(fontsize=15)
plt.grid(True)
plt.show()
# fig.savefig('lambda.pdf', dpi=2400, format='pdf')