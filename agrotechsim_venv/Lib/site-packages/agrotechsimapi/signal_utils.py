import numpy as np


def generate_gaussian_noise(num_samples, mean=0.0, stddev=1.0):
    
    noise = np.random.normal(mean, stddev, num_samples)
    return noise