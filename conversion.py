import cv2
import numpy as np
import csv
import matplotlib.pyplot as plt
import open3d as o3d

def generate_depth_map(image_path):
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    blurred = cv2.GaussianBlur(gray, (15, 15), 0)
    sobel_x = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=3)
    
    depth_map = np.sqrt(sobel_x**2 + sobel_y**2)
    
    depth_map = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
    depth_map = np.uint8(depth_map)
    
    save_depth_map_to_csv(depth_map, 'depth_map.csv')
    
    return depth_map

def save_depth_map_to_csv(depth_map, filename):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        for row in depth_map:
            writer.writerow(row)

def visualize_depth_map(depth_map):
    plt.imshow(depth_map, cmap='gray')
    plt.title('Depth Map')
    plt.show()

def create_3d_from_depth_map(image_path, depth_map):
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    
    height, width, _ = image.shape
    
    points = []
    for y in range(height):
        for x in range(width):
            depth = depth_map[y, x]
            color = image[y, x]
            
            point = [x, y, depth]
            points.append(point)
    points = np.array(points)
    
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    
    o3d.visualization.draw_geometries([point_cloud])


image_path = r'C:\Users\anups\OneDrive\Desktop\thesis\test.jpg'
depth_map = generate_depth_map(image_path)
visualize_depth_map(depth_map)
create_3d_from_depth_map(image_path, depth_map)
