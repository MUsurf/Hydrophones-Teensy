import math
import random
import numpy as np
from scipy.optimize import least_squares

# Constants
DHYDROPHONE = 0.5
SPEEDOFSOUND = 1480  # represents the speed of sound underwater (m/s)
# Hydrophone positions in an equilateral triangle
HYDROPHONES = np.array([
    [DHYDROPHONE, 0],   # Hydrophone 1 (reference)
    [-DHYDROPHONE/2, math.sqrt(3)* DHYDROPHONE/2],   # Hydrophone 2
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
    grid_x = np.linspace(0, 10, 20)  # 10 points along x-axis
    grid_y = np.linspace(0, 10, 20)  # 10 points along y-axis
    initial_guesses += [np.array([x, y]) for x in grid_x for y in grid_y]
    
    for guess in initial_guesses:
        result = least_squares(minimizationFunctions, guess, args=(d12, d13, d23), 
                              method='trf', ftol=1e-15, xtol=1e-15)
        
        if result.cost < best_cost:
            if(abs(result.x[0]) < 1 and abs(result.x[1] < 1)):
                continue
            best_cost = result.cost
            best_result = result
    
    return best_result

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
    estimated_point = result.x
    error_distance = distance_2d(test_point, estimated_point)
    
    # print(f"Original point: {test_point}")
    # print(f"Estimated point: {estimated_point}")
    # print(f"Error distance: {error_distance} meters")
    if( error_distance > 2):
        print("Error distance is greater than 10 meters")
    # print(f"Residual error: {result.cost}")
    
    return estimated_point, error_distance

def main():
    # Test with a known point
    for _ in range(100):
        test_point = [random.randint(-10, 10), random.randint(-10, 10)]
        test_accuracy(test_point)
        
        
    test_point = [random.randint(0,10), random.randint(0,10)]
    test_accuracy(test_point)
    
    # Test with point in different quadrant
    test_point3 = [-80, -60]
    test_accuracy(test_point3)
    
    

if __name__ == "__main__":
    main()