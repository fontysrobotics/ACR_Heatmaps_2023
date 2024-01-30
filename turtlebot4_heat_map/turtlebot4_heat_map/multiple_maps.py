import os
import png
import numpy as np
import glob
from datetime import date
   
# Generate a sequence of 2D arrays
def file_reader(file_name):
    array2d = []
    for row in open(file_name, 'r'):
        array2d.append(row.strip().split())

    yield array2d

# Collect all the pose maps for a given day
def collect_maps(height, width, map_name, cleaning_day):

    # Find the directory containing the pose matrices
    year      = str(cleaning_day.year)
    month     = str(cleaning_day.month)
    day       = str(cleaning_day.day)
    file_path = os.path.join(os.path.expanduser('~'), "cleaning_data", map_name, "pose_matrices", year, month, day)
    U21_array = np.zeros((height, width), dtype= int)

    # Check if the file exists, else return an empty pose map
    if os.path.isdir(file_path) == True:

        # Stack all the maps into one 3D array
        for file in glob.glob(os.path.join(file_path, '*txt')):
            file_name = os.path.join(os.getcwd(),file)
            for matrix in file_reader(file_name):
                U21_array = np.dstack((U21_array, matrix))
    else:
        U21_array = np.dstack((U21_array, U21_array))

    intarray = U21_array.astype(int)
    yield np.delete(intarray, 0, axis= 2)

def consolidate_arrays(pgmf, map_name, list_of_days):

    # Find the size of all the maps from the pgm file
    with open(pgmf,'rb') as f:
        size = str(f.readlines()[2:3][0])
        size = size[2:-3].split(" ")
        size = [int(i) for i in size]
        width, height = size

    multiple_paths = np.zeros((height, width),dtype= int)
    # Collect all the pose maps in a sequence of days
    for cleaning_day in list_of_days:
        for arrays in collect_maps(height, width, map_name, cleaning_day):
            
            # Split the arrays and add them element-wise
            split_matrices = np.split(arrays, arrays.shape[2], axis= 2)
            cleaning_array = np.sum(split_matrices, axis= 0)

            # Stack the consolidated arrays together
            multiple_paths = np.dstack((multiple_paths, cleaning_array))
    
    return np.delete(multiple_paths, 0, axis= 2)

def draw_png(pgmf, map_name, list_of_days, color_options):
    pose_matrix = consolidate_arrays(pgmf, map_name, list_of_days)
    map_matrix  = np.array([], int)

    # Find the size of the final 3D array, if only one map exists, set the number of maps to 1
    try:
        height, width, depth =  np.shape(pose_matrix)
    except ValueError:
        height, width, depth = (np.shape(pose_matrix), 1)

    # Initialize the matrix from the pgm map with integers as values
    with open(pgmf,'rb') as f:
        pgm_join = list(f.readlines()[4:])
        for line in pgm_join:
            for num in line:
                map_matrix = np.append(map_matrix,int(num))
    map_matrix = np.reshape(map_matrix, (height, width))

    img = []
    for y in range(int(height)):
            row = ()

            for x in range(int(width)):
                temp_row = [0, 0, 0]
                color_blend = []

                try:

                    # For each path going through a map cell, add a color to blend later
                    for z in range(depth):    
                        if int(pose_matrix[y,x,z]) > 0:
                            color_blend.append(color_options[z])
                    
                    # Color the map black for walls
                    if not color_blend or map_matrix[y, x] <= 10:
                        row = row + (int(map_matrix[y, x]), int(map_matrix[y, x]), int(map_matrix[y, x]))
                    
                    # Mix all the colors recorded for that cell together to make a new color
                    else:
                        for rgb in color_blend:
                                for value in range(3):
                                    temp_row[value] += int(rgb[value]/len(color_blend))
                        row = row + tuple(temp_row)
                except IndexError:

                    # Use only one color if only one map is loaded
                    if map_matrix[y, x] <= 10 or pose_matrix[y, x] == 0:
                        row = row + (int(map_matrix[y, x]), int(map_matrix[y, x]), int(map_matrix[y, x]))
                    else:
                        row = row + color_options[0]
            img.append(row)
    
    # Save the final png map
    with open(os.path.join(os.path.expanduser('~'), 'cleaning_data',map_name, 'heat_maps', str(list_of_days)+' multiple maps.png'), 'wb') as f:
        w = png.Writer(width, height, greyscale= False)
        w.write(f, img)

def main(list_of_days, map_name, color_options):
    pgmf = os.path.join(os.path.expanduser('~'), 'maps', map_name+'.pgm')
    draw_png(pgmf, map_name, list_of_days, color_options)

if __name__ == '__main__':
    date1 = date(2024, 1, 24)
    date2 = date(2024, 1, 23)
    date3 = date(2024, 1, 22)
    date4 = date(2024, 1, 21)
    date5 = date(2024, 1, 20)
    date6 = date(2024, 1, 17)
    date7 = date(2024, 1, 25)

    list_of_days  = [date7, date6]
    color_options = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255), (255, 255, 0)]

    main(map_name= 'office_1', list_of_days= list_of_days, color_options= color_options)