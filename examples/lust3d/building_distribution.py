import re
import numpy as np
from scipy.stats import beta

# Definition of parameters for the beta distribution
a, b, loc, scale = 2.202778570574113, 16.47770639852709, -1.1814002506076278, 138.03635306331512
np.random.seed(0)  # Set the seed for reproducibility

# Function to generate a new value for the layer parameter using beta distribution
def generate_new_layer_value():
    value = 0
    while value < 5:
        value = round(beta.rvs(a, b, loc=loc, scale=scale), 3)
    print(value)
    return value

# Read the file and replace the layer values
with open('lust3d.poly.xml', 'r') as f:
    content = f.read()

# Find all the values of the layer parameter
layers = re.findall(r'layer="([-+]?\d*\.\d+|[-+]?\d+)"', content)

# Replace the layer values with the newly generated values using beta distribution
new_content = re.sub(r'layer="([-+]?\d*\.\d+|[-+]?\d+)"', lambda x: f'layer="{generate_new_layer_value()}"', content)

# Write the new content to the file test.poly.3d.xml
with open('lust3d.poly.3d.xml', 'w') as f:
    f.write(new_content)

print("File test.poly.3d.xml generated successfully.")
