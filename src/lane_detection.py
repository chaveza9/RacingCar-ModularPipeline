import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
import skimage.color as color
import skimage.filters as filters
from scipy import ndimage as ndi
from skimage import feature

from scipy.optimize import minimize
import time


class LaneDetection:
    """
    Lane detection module using edge detection and b-spline fitting

    args: 
        cut_size (cut_size=68) cut the image at the front of the car
        spline_smoothness (default=10)
        gradient_threshold (default=14)
        distance_maxima_gradient (default=3)

    """

    def __init__(self, cut_size=68, spline_smoothness=0, gradient_threshold=20, distance_maxima_gradient=3):
        self.car_position = np.array([48, 0])
        self.spline_smoothness = spline_smoothness
        self.cut_size = cut_size
        self.gradient_threshold = gradient_threshold
        self.distance_maxima_gradient = distance_maxima_gradient
        self.lane_boundary1_old = 0
        self.lane_boundary2_old = 0

    def cut_gray(self, state_image_full):
        """"
        This function should cut the image at the front end of the car (e.g. pixel row 68)
        and translate to grey scale

        input:
            :param state_image_full 96x96x3

        output:
            :return gray_state_image 68x96x1

        """
        # Extract cutting limits
        cut_size = self.cut_size
        # Transform image to greyscale
        gray_state_image = color.rgb2grey(state_image_full)
        # Crop Information bar from image
        gray_state_image = gray_state_image[:cut_size, :]
        # Reshape image to a single channel
        #gray_state_image = gray_state_image.reshape(cut_size, 96, 1)
        return gray_state_image[::-1]

    def edge_detection(self, gray_image):
        """
        ##### TODO #####
        In order to find edges in the gray state image, 
        this function should derive the absolute gradients of the gray state image.
        Derive the absolute gradients using numpy for each pixel. 
        To ignore small gradients, set all gradients below a threshold (self.gradient_threshold) to zero. 

        input:
            :param gray_state_image 68x96x1

        output:
            :returns gradient_sum 68x96x1

        """
        filter_type = "canny"
        cut_size = self.cut_size
        # Compute image Gradient
        if filter_type == "sobel":
            gradient_sum = filters.sobel(gray_image)
            # Smooth out small gradients
            gradient_sum[gradient_sum < self.gradient_threshold/255] = 0
        elif filter_type == "canny":
            # Add gaussian blur
            gray_image = ndi.gaussian_filter(gray_image, 4)
            # Compute gradients
            gradient_sum = feature.canny(gray_image, sigma = 0)
            #gradient_sum = np.float64(gradient_sum)
        # Fill missing points
        gradient_sum[0, :] = gradient_sum[1, :]
        gradient_sum[cut_size - 1, :] = gradient_sum[cut_size - 2, :]

        return gradient_sum

    def find_maxima_gradient_rowwise(self, gradient_sum):
        """
        This function should output arguments of local maxima for each row of the gradient image.
        You can use scipy.signal.find_peaks to detect maxima. 
        Hint: Use distance argument for a better robustness.
        input:
            :param gradient_sum 68x96x1

        output:
            :returns maxima (np.array) 2x Number_maxima
        """
        arg_maxima = []
        for i in range(gradient_sum.shape[0]):
            arg_maxima.append(find_peaks(gradient_sum[i,:], distance=self.distance_maxima_gradient)[0])
        #arg_maxima= np.apply_along_axis(find_peaks, 1, gradient_sum, distance=self.distance_maxima_gradient)

        return arg_maxima

    def find_first_lane_point(self, gradient_sum):
        """
        Find the first lane_boundaries points above the car.
        Special cases like just detecting one lane_boundary or more than two are considered. 
        Even though there is space for improvement ;) 

        input:
            :param gradient_sum 68x96x1

        output: 
            :returns lane_boundary1_startpoint
            :returns lane_boundary2_startpoint
            :returns lanes_found  true if lane_boundaries were found
        """

        # Variable if lanes were found or not
        lanes_found = False
        row = 0

        # loop through the rows
        while not lanes_found:

            # Find peaks with min distance of at least 3 pixel 
            argmaxima = find_peaks(gradient_sum[row], distance=3)[0]

            # if one lane_boundary is found
            if argmaxima.shape[0] == 1:
                lane_boundary1_startpoint = np.array([[argmaxima[0], row]])

                if argmaxima[0] < 48:
                    lane_boundary2_startpoint = np.array([[0, row]])
                else:
                    lane_boundary2_startpoint = np.array([[96, row]])

                lanes_found = True

            # if 2 lane_boundaries are found
            elif argmaxima.shape[0] == 2:
                lane_boundary1_startpoint = np.array([[argmaxima[0], row]])
                lane_boundary2_startpoint = np.array([[argmaxima[1], row]])
                lanes_found = True

            # if more than 2 lane_boundaries are found
            elif argmaxima.shape[0] > 2:
                # if more than two maxima then take the two lanes next to the car, regarding least square
                A = np.argsort((argmaxima - self.car_position[0]) ** 2)
                lane_boundary1_startpoint = np.array([[argmaxima[A[0]], 0]])
                lane_boundary2_startpoint = np.array([[argmaxima[A[1]], 0]])
                lanes_found = True

            row += 1

            # if no lane_boundaries are found
            if row == self.cut_size:
                lane_boundary1_startpoint = np.array([[0, 0]])
                lane_boundary2_startpoint = np.array([[0, 0]])
                break

        return lane_boundary1_startpoint, lane_boundary2_startpoint, lanes_found

    def lane_detection(self, state_image_full):
        """
        This function should perform the road detection 

        args:
            :param state_image_full [96, 96, 3]

        out:
            :return lane_boundary1 spline
            :return lane_boundary2 spline
        """

        # to gray
        gray_state = self.cut_gray(state_image_full)

        # edge detection via gradient sum and thresholding
        gradient_sum = self.edge_detection(gray_state)
        maxima = self.find_maxima_gradient_rowwise(gradient_sum)

        # first lane_boundary points
        lane_boundary1_points, lane_boundary2_points, lane_found = self.find_first_lane_point(gradient_sum)

        old_point_1 = lane_boundary1_points
        old_point_2 = lane_boundary2_points
        row = 0

        #lane_boundary1_points = np.empty((0,2), int)
        #lane_boundary2_points = np.empty((0,2), int)


        # if no lane was found,use lane_boundaries of the preceding step
        if lane_found:

            ##### TODO #####
            #  in every iteration: 
            # 1- find maximum/edge with the lowest distance to the last lane boundary point 
            # 2- append maximum to lane_boundary1_points or lane_boundary2_points
            # 3- delete maximum from maxima
            # 4- stop loop if there is no maximum left 
            #    or if the distance to the next one is too big (>=100)
            # Iterate through maxima
            while 0 <= row < self.cut_size:
                if maxima[row].shape[0] != 0:
                    # Form coordinate list
                    edge_points = []
                    for edge in maxima[row]:
                        edge_points.append(np.array([edge, row]))
                    #print(edge_points)
                    # lane_boundary 1
                    closest_point_1, dist_1 = self.closest_node(old_point_1, edge_points)
                    # lane_boundary 2
                    closest_point_2, dist_2 = self.closest_node(old_point_2, edge_points)
                    # Delete maximum
                    maxima[row] = np.delete(maxima[row], np.where(maxima[row] == closest_point_1[0, 0]))
                    maxima[row] = np.delete(maxima[row], np.where(maxima[row] == closest_point_2[0, 0]))
                    if (20 >= dist_1 > 0) and (20 >= dist_2 > 0):
                        # Assign new values
                        old_point_1 = closest_point_1
                        old_point_2 = closest_point_2
                        # Append lane values
                        lane_boundary1_points = np.concatenate((lane_boundary1_points, old_point_1))
                        lane_boundary2_points = np.concatenate((lane_boundary2_points, old_point_2))
                    row += 1
                else:
                    row -= 1


            # spline fitting using scipy.interpolate.splprep 
            # and the arguments self.spline_smoothness
            # 
            # if there are more lane_boundary points points than spline parameters 
            # else use perceding spline
            if lane_boundary1_points.shape[0] > 4 and lane_boundary2_points.shape[0] > 4:
                # Pay attention: the first lane_boundary point might occur twice
                # Compute uniques
                lane_boundary1_points = np.unique(lane_boundary1_points, axis=0)
                # lane_boundary 1
                # Compute spline
                x_1 = np.float64(lane_boundary1_points[:, 0])
                y_1 = np.float64(lane_boundary1_points[:, 1])
                lane_boundary1, _ = splprep([x_1, y_1], s=self.spline_smoothness)
                # lane_boundary 2
                lane_boundary2_points = np.unique(lane_boundary2_points, axis=0)
                x_2 = np.float64(lane_boundary2_points[:, 0])
                y_2 = np.float64(lane_boundary2_points[:, 1])
                lane_boundary2, _ = splprep([x_2, y_2], s=self.spline_smoothness)

            else:
                lane_boundary1 = self.lane_boundary1_old
                lane_boundary2 = self.lane_boundary2_old
            ################

        else:
            lane_boundary1 = self.lane_boundary1_old
            lane_boundary2 = self.lane_boundary2_old

        self.lane_boundary1_old = lane_boundary1
        self.lane_boundary2_old = lane_boundary2

        # output the spline
        return lane_boundary1, lane_boundary2

    def closest_node(self, point, points):
        points = np.asarray(points)
        dist_2 = np.sum((points - point) ** 2, axis=1)
        min_indx = np.argmin(dist_2)
        return points[min_indx: min_indx+1, :], dist_2[min_indx]

    def plot_state_lane(self, state_image_full, steps, fig, waypoints=[]):
        """
        Plot lanes and way points
        """
        # evaluate spline for 6 different spline parameters.
        t = np.linspace(0, 1, 6)
        lane_boundary1_points_points = np.array(splev(t, self.lane_boundary1_old))
        lane_boundary2_points_points = np.array(splev(t, self.lane_boundary2_old))

        plt.gcf().clear()
        plt.imshow(state_image_full[::-1])
        plt.plot(lane_boundary1_points_points[0], lane_boundary1_points_points[1] + 96 - self.cut_size, linewidth=5,
                 color='orange')
        plt.plot(lane_boundary2_points_points[0], lane_boundary2_points_points[1] + 96 - self.cut_size, linewidth=5,
                 color='orange')
        if len(waypoints):
            plt.scatter(waypoints[0], waypoints[1] + 96 - self.cut_size, color='white')

        #plt.axis('off')
        plt.xlim((-0.5, 95.5))
        plt.ylim((-0.5, 95.5))
        plt.gca().axes.get_xaxis().set_visible(True)
        plt.gca().axes.get_yaxis().set_visible(True)
        fig.canvas.flush_events()
