import numpy as np
import matplotlib.pyplot as plt

# Parameters
num_particles = 500  # Number of particles used in the filter
world_size = 100.0  # Size of the world for boundary conditions
initial_position = [0.0, 0.0]  # Starting position at the origin
motion_noise_std = [1.0, 1.0]  # Standard deviation of noise in particle motion
measurement_noise_std = [2.0, 2.0]  # Standard deviation of noise in measurements

# Initialize particles randomly around the initial position
particles = np.random.randn(num_particles, 2) * motion_noise_std + initial_position
weights = np.ones(num_particles) / num_particles  # Initialize weights uniformly

def predict(particles, control, noise_std):
    """
    Move particles based on control input and noise.
    :param particles: Array of particle positions
    :param control: Control input [dx, dy]
    :param noise_std: Standard deviation of the motion noise
    :return: Updated particle positions
    """
    # Move particles according to control input and add noise
    particles += control + np.random.randn(num_particles, 2) * noise_std
    particles %= world_size  # Keep particles within the world boundaries
    return particles

def update(particles, weights, measurement, measurement_std):
    """
    Update particle weights based on the measurement.
    :param particles: Array of particle positions
    :param weights: Array of particle weights
    :param measurement: Actual position measurement
    :param measurement_std: Standard deviation of the measurement noise
    :return: Updated particle weights
    """
    # Calculate the distance between each particle and the measurement
    dist = np.linalg.norm(particles - measurement, axis=1)
    # Update weights based on the distance (likelihood)
    weights *= np.exp(-0.5 * (dist / measurement_std[0])**2)
    weights += 1.e-300  # Avoid weights becoming zero
    weights /= np.sum(weights)  # Normalize weights
    return weights

def resample(particles, weights):
    """
    Resample particles based on their weights.
    :param particles: Array of particle positions
    :param weights: Array of particle weights
    :return: Resampled particles
    """
    # Choose particle indices based on weights
    indices = np.random.choice(np.arange(num_particles), num_particles, p=weights)
    return particles[indices]

def particle_filter_simulation(num_steps, control_inputs, true_positions):
    """
    Run the particle filter simulation.
    :param num_steps: Number of time steps to simulate
    :param control_inputs: Control inputs for each time step
    :param true_positions: True positions of the object
    :return: True positions and estimated positions from the filter
    """
    global particles, weights
    estimated_positions = []  # Store the filtered positions

    for step in range(num_steps):
        # Predict the next state of particles
        particles = predict(particles, control_inputs[step], motion_noise_std)
        
        # Update weights based on the true position
        weights = update(particles, weights, true_positions[step], measurement_noise_std)
        
        # Resample particles according to the updated weights
        particles = resample(particles, weights)
        
        # Compute the estimated position as the mean of particles
        estimated_positions.append(np.mean(particles, axis=0))
    
    return np.array(true_positions), np.array(estimated_positions)

# Generate synthetic data for the simulation
num_steps = 50  # Number of simulation steps
true_positions = np.zeros((num_steps, 2))  # Initialize true positions array
control_inputs = np.zeros((num_steps, 2))  # Initialize control inputs array

# Generate control inputs and true positions
for step in range(1, num_steps):
    control_inputs[step] = [1.0, 1.0]  # Control input: move along the line x=y
    true_positions[step] = true_positions[step-1] + control_inputs[step] + np.random.randn(2) * measurement_noise_std

# Apply the particle filter
true_positions, estimated_positions = particle_filter_simulation(num_steps, control_inputs, true_positions)

# Plot the results
plt.figure(figsize=(12, 6))
plt.plot(true_positions[:, 0], true_positions[:, 1], 'g', label='True Positions', marker='o')
plt.plot(estimated_positions[:, 0], estimated_positions[:, 1], 'r', label='Filtered Positions', marker='x')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('2D Object Movement: True vs Filtered Positions (Moving along x=y line)')
plt.legend()
plt.grid()
plt.show()
