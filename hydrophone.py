import math
import random
import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# Constants
DHYDROPHONE = 0.5
SPEEDOFSOUND = 1480  # represents the speed of sound underwater (m/s)
# Hydrophone positions in an equilateral triangle
HYDROPHONES = np.array([
    [DHYDROPHONE+.1, 0],   # Hydrophone 1 (reference)
    [-DHYDROPHONE/2, math.sqrt(3)* DHYDROPHONE/2 -.2],   # Hydrophone 2
    [-DHYDROPHONE/2, -math.sqrt(3) * DHYDROPHONE/2]    # Hydrophone 3
])

# Convert the time differences to distance differences
def convertToDistanceDifferences(tdoa_12, tdoa_13, tdoa_23):
    """Convert time differences to distance differences
    tdoa_12: time difference between hydrophone 1 and 2 (t2-t1)
    tdoa_13: time difference between hydrophone 1 and 3 (t3-t1)
    tdoa_23: time difference between hydrophone 2 and 3 (t3-t2)
    """
    return (tdoa_12*SPEEDOFSOUND, tdoa_13*SPEEDOFSOUND, tdoa_23*SPEEDOFSOUND)

# Residual function for least squares optimization
def minimizationFunctions(position, d12, d13, d23):
    """Calculate residuals for least squares optimization
    position: [x, y] - current position estimate
    d12, d13, d23: distance differences between hydrophones
    """
    x, y = position
    # Calculate distances from source to each hydrophone
    d1 = np.sqrt((x - HYDROPHONES[0,0])**2 + (y - HYDROPHONES[0,1])**2)
    d2 = np.sqrt((x - HYDROPHONES[1,0])**2 + (y - HYDROPHONES[1,1])**2)
    d3 = np.sqrt((x - HYDROPHONES[2,0])**2 + (y - HYDROPHONES[2,1])**2)
    
    # Calculate residuals (how well the current position matches measured distance differences)
    return [
        (d2 - d1) - d12,  # Time difference 1-2
        (d3 - d1) - d13,  # Time difference 1-3
        (d3 - d2) - d23   # Time difference 2-3
    ]

def calculations(time1, time2, time3):
    # Time differences of arrival (TDOA)
    tdoa_12 = time2 - time1  # Time difference between H1 and H2
    tdoa_13 = time3 - time1  # Time difference between H1 and H3
    tdoa_23 = time3 - time2  # Time difference between H2 and H3

    # Convert time differences to distance differences
    d12, d13, d23 = convertToDistanceDifferences(tdoa_12, tdoa_13, tdoa_23)
    
    # Calculate center of hydrophone array
    hydrophone_center = np.mean(HYDROPHONES, axis=0)
    
    # Try multiple initial guesses to avoid local minima
    best_result = None
    best_cost = float('inf')
    
    # Define several initial guesses including far field estimates
    initial_guesses = [
        hydrophone_center
    ]
    grid_x = np.linspace(-10.5, 10.5, 20)  # 10 points along x-axis
    grid_y = np.linspace(-10.5, 10.5, 20)  # 10 points along y-axis
    
    for x in grid_x:
        for y in grid_y:
            initial_guesses.append(np.array([x, y]))
    
    for guess in initial_guesses:
        result = least_squares(minimizationFunctions, guess, args=(d12, d13, d23), 
                              method='trf', ftol=1e-15, xtol=1e-15,
                              bounds=([-11, -11], [11, 11]))
        
        if result.cost < best_cost:
            if(abs(result.x[0]) < 1 and abs(result.x[1] < 1)):
                continue
            best_cost = result.cost
            best_result = result
    
    # Visualize the solution if we found one
    if best_result is not None:
        visualize_solution(best_result.x, time1, time2, time3)
    
    return best_result

def visualize_solution(estimated_point, time1, time2, time3):
    """
    Visualize the estimated position and the three circles from each hydrophone.
    
    Args:
        estimated_point (array): The estimated [x, y] position
        time1, time2, time3 (float): Arrival times at each hydrophone
    """
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Calculate distances from each hydrophone to the estimated point
    distances = []
    for i in range(3):
        distances.append(distance_2d(HYDROPHONES[i], estimated_point))
    
    # Plot hydrophones as points
    for i, (x, y) in enumerate(HYDROPHONES):
        ax.plot(x, y, 'bo', markersize=10, label=f'Hydrophone {i+1}')
        
    # Plot estimated position
    ax.plot(estimated_point[0], estimated_point[1], 'r*', markersize=15, label='Estimated Position')
    
    # Draw circles representing distances
    colors = ['blue', 'green', 'purple']
    for i in range(3):
        circle = Circle(HYDROPHONES[i], distances[i], fill=False, color=colors[i], 
                       linestyle='--', alpha=0.7, label=f'Distance to H{i+1}')
        ax.add_patch(circle)
    
    # Set axis labels and title
    ax.set_xlabel('X position (m)')
    ax.set_ylabel('Y position (m)')
    ax.set_title('Hydrophone Localization Visualization')
    
    # Adjust axis limits to show all relevant elements
    buffer = max(distances) * 0.2  # 20% extra space
    xlim = [min(min(HYDROPHONES[:,0]), estimated_point[0]) - buffer, 
            max(max(HYDROPHONES[:,0]), estimated_point[0]) + buffer]
    ylim = [min(min(HYDROPHONES[:,1]), estimated_point[1]) - buffer, 
            max(max(HYDROPHONES[:,1]), estimated_point[1]) + buffer]
    
    # Make sure we see the hydrophone array even if source is far away
    hydrophone_size = np.max(HYDROPHONES) - np.min(HYDROPHONES)
    min_view_size = hydrophone_size * 2  # Ensure we see at least this much
    
    if xlim[1] - xlim[0] < min_view_size:
        center = (xlim[0] + xlim[1]) / 2
        xlim = [center - min_view_size/2, center + min_view_size/2]
    
    if ylim[1] - ylim[0] < min_view_size:
        center = (ylim[0] + ylim[1]) / 2
        ylim = [center - min_view_size/2, center + min_view_size/2]
    
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    
    # Add legend and grid
    ax.legend(loc='upper right')
    ax.grid(True)
    
    # Add info about arrival times and distances
    time_info = f"Arrival times: H1={time1:.6f}s, H2={time2:.6f}s, H3={time3:.6f}s"
    distance_info = f"Distances: H1={distances[0]:.2f}m, H2={distances[1]:.2f}m, H3={distances[2]:.2f}m"
    tdoa_info = f"TDOA: H2-H1={(time2-time1)*1000:.3f}ms, H3-H1={(time3-time1)*1000:.3f}ms, H3-H2={(time3-time2)*1000:.3f}ms"
    
    plt.figtext(0.5, 0.01, time_info + "\n" + distance_info + "\n" + tdoa_info, 
                ha="center", fontsize=9, bbox={"facecolor":"orange", "alpha":0.2, "pad":5})
    
    # Show plot
    plt.tight_layout(rect=[0, 0.05, 1, 0.95])  # Adjust layout to make room for text
    plt.show()

def distance_2d(point1, point2):
    """Calculates the Euclidean distance between two points in 2D space."""
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

def convertPointToTimes(point):
    """Calculate arrival times for a sound originating at the given point."""
    times = [0, 0, 0]
    for i in range(3):
        times[i] = distance_2d(HYDROPHONES[i], point) / SPEEDOFSOUND
    return times

def test_accuracy(test_point):
    """Test the accuracy of our algorithm with a known point."""
    print(f"Testing with point: {test_point}")
    
    # Calculate the expected times of arrival
    times = convertPointToTimes(test_point)
    
    # Run our localization algorithm
    result = calculations(times[0], times[1], times[2])
    
    # Calculate error
    if(result is None):
        print("No result found")
        return None
    estimated_point = result.x
    error_distance = distance_2d(test_point, estimated_point)
    
    if( error_distance > 2):
        print("Error distance is greater than 2 meters")
        print("estimated point: ", estimated_point)
    
    # Plot the true point along with the estimated point for comparison
    compare_true_and_estimated(test_point, estimated_point, times)
    
    return estimated_point, error_distance

def compare_true_and_estimated(true_point, estimated_point, times):
    """
    Plot both the true and estimated points with their respective distance circles.
    
    Args:
        true_point: The actual source location
        estimated_point: The calculated source location
        times: Arrival times at each hydrophone
    """
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Calculate distances from each hydrophone to both points
    true_distances = []
    est_distances = []
    for i in range(3):
        true_distances.append(distance_2d(HYDROPHONES[i], true_point))
        est_distances.append(distance_2d(HYDROPHONES[i], estimated_point))
    
    # Plot hydrophones
    for i, (x, y) in enumerate(HYDROPHONES):
        ax.plot(x, y, 'bo', markersize=10, label=f'Hydrophone {i+1}' if i==0 else "")
    
    # Plot true position
    ax.plot(true_point[0], true_point[1], 'g*', markersize=15, label='True Position')
    
    # Plot estimated position
    ax.plot(estimated_point[0], estimated_point[1], 'r*', markersize=15, label='Estimated Position')
    
    # Draw circles for true distances (solid lines)
    for i in range(3):
        circle = Circle(HYDROPHONES[i], true_distances[i], fill=False, color='green', 
                       linestyle='-', alpha=0.5)
        ax.add_patch(circle)
    
    # Draw circles for estimated distances (dashed lines)
    for i in range(3):
        circle = Circle(HYDROPHONES[i], est_distances[i], fill=False, color='red', 
                       linestyle='--', alpha=0.7)
        ax.add_patch(circle)
    
    # Set axis labels and title
    ax.set_xlabel('X position (m)')
    ax.set_ylabel('Y position (m)')
    ax.set_title('True vs Estimated Position')
    
    # Calculate appropriate axis limits
    all_points = np.vstack([HYDROPHONES, [true_point], [estimated_point]])
    all_distances = true_distances + est_distances
    max_dist = max(all_distances)
    
    x_min, y_min = np.min(all_points, axis=0) - max_dist*0.2
    x_max, y_max = np.max(all_points, axis=0) + max_dist*0.2
    
    ax.set_xlim([x_min, x_max])
    ax.set_ylim([y_min, y_max])
    
    # Add legend and grid
    ax.legend(loc='upper right')
    ax.grid(True)
    
    # Add error info
    error = distance_2d(true_point, estimated_point)
    error_text = f"Error distance: {error:.3f}m"
    plt.figtext(0.5, 0.01, error_text, ha="center", fontsize=12, 
               bbox={"facecolor":"orange", "alpha":0.5, "pad":5})
    
    plt.tight_layout(rect=[0, 0.05, 1, 0.95])
    plt.show()

def main():
    # Test with a single known point for visualization
    # test_point = [5, 7]  # Choose a specific test point for visualization
    # test_accuracy(test_point)
    
    # Uncomment to run multiple random tests
    # for _ in range(5):  # Reduced from 100 to 5 tests to prevent too many plots
    #     test_point = [random.randint(-10, 10), random.randint(-10, 10)]
    #     test_accuracy(test_point)
    
    test_accuracy([9,4])
    test_accuracy([-8,6])

if __name__ == "__main__":
    main()