#!/usr/bin/python3
import sys
import re, numpy as np
import matplotlib.pyplot as plt

colors = ["blue", "green", "purple", "magenta", "lime", "pink", "brown", "olive", "navy"]

def tri_is_winding_cc(pts):
    vec1 = np.concatenate([pts[1] - pts[0], np.zeros(1)], 0)
    vec2 = np.concatenate([pts[2] - pts[0], np.zeros(1)], 0)
    cross_product = np.cross(vec1, vec2)

    return cross_product[2] >= 0

def plot(triangles, points):
    plt.figure()
    
    for i, triangle in enumerate(triangles):
        if tri_is_winding_cc(triangle):
            color = colors[i % len(colors)]
            alpha = 0.4
        else:
            color = "red"
            alpha = 1.0

        plt.gca().scatter(triangle[0][0], triangle[0][1], color=color, alpha=alpha)
        plt.gca().annotate(f"t{i}", triangle[0])
        plt.gca().add_patch(plt.Polygon(triangle, color=color, alpha=alpha))

    for i, point in enumerate(points):
        plt.gca().scatter([point[0]], [point[1]], color="red", s=5)
        plt.gca().annotate(f"p{i}", point)

    plt.autoscale(True)
    plt.show()

def get_numbers(str):
    numbers = re.findall('-?(?:\d+(?:\.\d+)?(?:e-?\d+)?)|inf|nan', str)
    numbers = np.array([float(n) for n in numbers])
    return numbers

if __name__ == "__main__":
    triangle_list = get_numbers(sys.argv[1]).reshape((-1, 3, 2))

    if len(sys.argv) > 2:
        point_list = get_numbers(sys.argv[2]).reshape((-1, 2))
    else:
        point_list = []

    print("triangles: ", triangle_list, "points: ", point_list)
    plot(triangle_list, point_list)
