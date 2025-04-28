import numpy as np
from sys import argv

def max_distance_without_intersection(midpoint, curve, max_dist):
    distances = np.linalg.norm(curve - midpoint, axis=1)
    min_distance = np.min(distances)
    return min(min_distance, max_dist)

def compute_centerline_sphere(curve1, curve2):
    centerline = []

    for point in curve1:

        distances = np.linalg.norm(curve2 - point, axis=1)
        closest_idx = np.argmin(distances)
        closest_point = curve2[closest_idx]
        
        midpoint = (point + closest_point) / 2

        r1 = np.linalg.norm(point - midpoint)
        r2 = max_distance_without_intersection(midpoint, curve2, r1)

        direction = (midpoint - point) / r1
        new_point = point + direction * r2

        centerline.append(new_point)

    return np.array(centerline)


if __name__ == "__main__":
    if len(argv) != 4:
        print('Usage: python find_center_line_2.py <outer_boundary_csv> <inner_boundary_csv> <output_csv>')

    outer_boundary_csv = argv[1]
    inner_boundary_csv = argv[2]
    output_csv = argv[3]

    curve1 = np.loadtxt(outer_boundary_csv, delimiter=',', skiprows=1)
    curve2 = np.loadtxt(inner_boundary_csv, delimiter=',', skiprows=1)

    centerline = compute_centerline_sphere(curve1, curve2)


    np.savetxt(output_csv, centerline, delimiter=',', header='x,y,z', comments='')

    import matplotlib.pyplot as plt
    plt.plot(curve1[:, 0], curve1[:, 1], 'r')
    plt.plot(curve2[:, 0], curve2[:, 1], 'b')
    plt.plot(centerline[:, 0], centerline[:, 1], 'g')
    plt.legend(['Curve 1', 'Curve 2', 'Centerline'])
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()
