from datetime import date, timedelta
import os
import glob
import png
import numpy as np

# Initialize a matrix representing the map with integers as values
def open_pgm(pgmf, array_3d):
    height, width, depth = array_3d.shape
    [depth]

    map_matrix = np.array([], dtype= int)
    with open(pgmf,'rb') as f:
        pgm_join = list(f.readlines()[4:])

    for line in pgm_join:
        for num in line:
            map_matrix = np.append(map_matrix, int(num))

    return np.reshape(map_matrix, (height, width))

# Map an input range to an output range
def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min)*(out_max - out_min)//(in_max - in_min)+out_min

# Return a 2D matrix from a file name
def file_reader(file_name):
    array2d = []
    for row in open(file_name, 'r'):
        num = row.strip().split()
        array2d.append(num)

    yield array2d

# Return a sequence of years, months and days between two date ranges
def gen_dates(end, date_difference):

    for days in range(date_difference.days+1):
        cleaning_day = end - timedelta(days)
        year = str(cleaning_day.year)
        month = str(cleaning_day.month)
        day = str(cleaning_day.day)
        yield year, month, day

# Collect all the pose maps of one day
def collect_maps(height, width, map_name, cleaning_day):
    map_count = 0

    # Find the directory containing the pose matrices
    year, month, day = cleaning_day
    file_path = os.path.join(os.path.expanduser('~'), "cleaning_data", map_name, "pose_matrices", year, month, day)
    U21_array = np.zeros((height, width), dtype= int)

    # Check if the file exists, else return an empty pose map
    if os.path.isdir(file_path) == True:
        for file in glob.glob(os.path.join(file_path, '*txt')):
            # Increase the map counter
            map_count += 1
            file_name = os.path.join(os.getcwd(),file)

            # Stack all the collected maps into a 3D array
            for matrix in file_reader(file_name):
                U21_array = np.dstack((U21_array, matrix))
    else:
        U21_array = np.dstack((U21_array, U21_array))

    intarray = U21_array.astype(int)
    yield np.delete(intarray, 0, axis= 2), map_count

def consolidate_arrays(pgmf, map_name, start, end):
    map_count = 0
    date_difference = end - start

    # Get the size of the map from the pgm file
    with open(pgmf,'rb') as f:
        size = str(f.readlines()[2:3][0])
        size = size[2:-3].split(" ")
        size = [int(i) for i in size]
        width, height = size


    multiple_paths = np.zeros((height, width),dtype= int)
    # Collect all the pose maps from each day between the start and end date
    for cleaning_day in gen_dates(end, date_difference):
        for arrays, map_counter in collect_maps(height, width, map_name, cleaning_day):
            # Add the map counter for a specific day to the total map counter
            map_count += map_counter

            # Split the arrays and add them element-wise
            split_matrices = np.split(arrays, arrays.shape[2], axis= 2)
            cleaning_array = np.sum(split_matrices, axis= 0)

            # Stack the consolidated arrays together
            multiple_paths = np.dstack((multiple_paths, cleaning_array))
    
    return np.delete(multiple_paths, 0, axis= 2), map_count

def recency_heatmap(array_3d, map_matrix, map_name, start, end):
    height, width, depth = np.shape(array_3d)

    # Initialize a matrix for the png map, starting with the longest day as its initial values
    cleaning_array = np.full((height, width), depth)
    
    # Replace the values in the initialized map with the shortest relative date length where it has been cleaned
    indices_3d = np.argwhere(array_3d)
    for z, y, x in indices_3d:
        cleaning_array[z, y] = x if x < cleaning_array[z, y] else cleaning_array[z, y]
        [x]
    
    # Set the color values on the map
    img = []
    for y in range(height):
            row = ()

            for x in range(width):
                # Color the walls black
                if map_matrix[y, x] <= 10 or cleaning_array[y, x] == int(depth):
                    row = row + (int(map_matrix[y, x]), int(map_matrix[y, x]), int(map_matrix[y, x]))
                # Color light blue to teal for clean
                elif cleaning_array[y, x] <= 0.2*depth:
                    blue_to_teal = map_range(cleaning_array[y, x], 0, 0.2*depth, 60, 255)
                    row = row + (0, int(blue_to_teal),255)
                # Color from teal to green for pretty clean
                elif cleaning_array[y, x] <= 0.45*depth:
                    teal_to_green = map_range(cleaning_array[y, x], 0.2*depth, 0.45*depth, 0, 255)
                    row = row + (0, 255, 255-int(teal_to_green))
                # Color from green to yellow for not so clean
                elif cleaning_array[y, x] <= 0.7*depth:
                    green_to_yellow = map_range(cleaning_array[y, x], 0.45*depth, 0.7*depth, 0, 255)
                    row = row + (int(green_to_yellow), 255, 0)
                # Color from yellow to red for not clean
                elif cleaning_array[y, x] <= depth:
                    yellow_to_red = map_range(cleaning_array[y, x], 0.7*depth, depth, 0, 135)
                    row = row + (255, 255-int(yellow_to_red), 0)
                else:
                    row = row + (250 ,120 ,0)
            img.append(row)

    # Save the png with the map name and dates in mind
    with open(os.path.join(os.path.expanduser('~'), 'cleaning_data', map_name, 'heat_maps', str(start)+' to '+str(end)+' recent.png'), 'wb') as f: 
        w = png.Writer(width, height, greyscale= False)
        w.write(f, img)

def history_heatmap(array_3d, map_matrix, map_name, start, end):
    height, width, depth = np.shape(array_3d)
    
    """
    # Use the pose range from the pose map with a max value of 30 to make the cleanliness count
    cleaning_array = np.zeros((height, width), dtype= int)
    for sub_matrix in array_3d:
        cleaning_array += sub_matrix"""

    # Use a minimal pose count of 1 from the pose map to make the cleanliness count
    cleaning_array = np.zeros((height, width), dtype= int)
    indices_3d = np.argwhere(array_3d)
    for z,y,x in indices_3d:
        cleaning_array[z, y] += 1
    [x]

    # Set the color values on the png
    img = []
    for y in range(height):
            row = ()

            for x in range(width):
                # Color the walls black
                if map_matrix[y, x] <= 10 or cleaning_array[y, x] == 0:
                    row = row + (int(map_matrix[y,x]), int(map_matrix[y, x]), int(map_matrix[y,x]))
                # Color from red to yellow for not cleaned
                elif cleaning_array[y, x] <= 0.2*depth:
                    red_to_yellow = map_range(cleaning_array[y, x], -1, 0.2*depth, 0, 255)
                    row = row + (255, int(red_to_yellow),0)
                # Color from yellow to green for not so clean
                elif cleaning_array[y, x] <= 0.45*depth:
                    yellow_to_green = map_range(cleaning_array[y, x], 0.2*depth, 0.45*depth, 0, 255)
                    row = row + (255-int(yellow_to_green),255,0)
                # Color from green to teal for pretty clean
                elif cleaning_array[y, x] <= 0.7*depth:
                    green_to_teal = map_range(cleaning_array[y, x], 0.45*depth, 0.7*depth, 0, 255)
                    row = row + (0, 255, int(green_to_teal))
                # Color from teal to blue for clean
                elif cleaning_array[y,x] <= depth:
                    teal_to_blue = map_range(cleaning_array[y, x], 0.7*depth, depth, 0, 135)
                    row = row + (0, 255-int(teal_to_blue), 255)
                else:
                    row = row + (0, 120, 255)
            img.append(row)
    
    # Save the png using the map name and date range
    with open(os.path.join(os.path.expanduser('~'), 'cleaning_data', map_name, 'heat_maps', str(start)+' to '+str(end)+' over time.png'), 'wb') as f:
        w = png.Writer(width, height, greyscale= False)
        w.write(f, img)

def main(start, end, map_name):
    pgmf= os.path.join(os.path.expanduser('~'),'maps', map_name+'.pgm')

    array_3d, map_count= consolidate_arrays(pgmf, map_name, start, end)
    map_matrix= open_pgm(pgmf, array_3d)
    history_heatmap(array_3d, map_matrix, map_name, start, end)
    recency_heatmap(array_3d, map_matrix, map_name, start, end)

    return map_count

if __name__ == '__main__':
    # Test case
    start = date(2024,1,17)
    end = date(2024,1,23)
    main(start,end,map_name='office_1')
