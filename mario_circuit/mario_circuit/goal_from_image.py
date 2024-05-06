import cv2
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, height = 600, width = 800, template = None):
    """
    Implement the cone detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected. BGR.
        template_file_path; Not required, but can optionally be used to automate setting hue filter values.
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox"""
    
    ########## YOUR CODE STARTS HERE ##########
	
    height_of_interest = int(height * .625)
    width_low = int(width*.125)
    width_high = int(width*.875)
    w = 25
    kernel_size = 15
    threshold = 210
    mask = cv2.inRange(img, np.array([threshold, threshold, threshold]), np.array([255, 255, 255]))
    result = cv2.bitwise_and(img, img, mask=mask)
    kernel = np.ones((kernel_size, kernel_size))
    kernel2 = np.ones((39, 39))
    img_threshold = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel)
    img_thick = cv2.dilate(img_threshold, kernel2)
    _, thresholded = cv2.threshold(img_thick, threshold, 255, cv2.THRESH_BINARY)
    row = thresholded[height_of_interest][width_low:width_high]
    white_indices = np.where(row == 255)[0]
    if len(white_indices) > 0:
        average_x = width_low + int(np.mean(white_indices))
        # Uncomment for debugging
        # cv2.rectangle(img,  (average_x - w, height_of_interest - w), (average_x + w, height_of_interest + w), (0, 255, 0), 2)
        # cv2.rectangle(img_thick, (width_low, height_of_interest - w), (width_high, height_of_interest + w), (0, 255, 0), 2)


    # Uncomment this line for debugging
    # image_print(img_thick)
    # image_print(img)
    return (average_x, height_of_interest)
	    
if __name__ == "__main__":
    # code in this block will only be run when you explicitly run your script,
    # and not when the tests are being run.  this is a good place for
    # generating images, etc.
	img = cv2.imread("/Users/ericscomputer/racecar_docker/home/racecar_ws/src/final_challenge/final_challenge2024/mario_circuit/track_test_pics/end_corner_outer.png")
	print(cd_color_segmentation(img))
     
    # pass
	


