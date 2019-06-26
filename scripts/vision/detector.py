import numpy as np
import cv2 as cv
import rospy

from bronkhorst.msg import LfeCoordinate

MIN_MATCH_COUNT = 10
FOV_WIDTH = 0.2510
FOV_HEIGHT = 0.1585
FOV_PIX_X = 1936.0
FOV_PIX_Y = 1216.0


def detect_circles(img):
    img = cv.medianBlur(img, 5)
    rows = img.shape[0]
    circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, 1, rows / 8,
                              param1=90, param2=20,
                              minRadius=35, maxRadius=45)  # min max height is dependent on camera height
    if circles is not None:
        return np.uint16(np.around(circles))
    else:
        return None


def crop_circle_img(img, circle, margin):
    x1 = circle[1] - circle[2] - margin
    x2 = circle[1] + circle[2] + margin
    y1 = circle[0] - circle[2] - margin
    y2 = circle[0] + circle[2] + margin
    cropped_img = img[x1:x2, y1:y2]
    return cropped_img


def sif_feature_detection(test_img):
    template = cv.imread('/home/wybren/catkin_ws/src/bronkhorst/template/upside.jpeg', 0)
    orb = cv.ORB_create()
    kp1, des1 = orb.detectAndCompute(template, None)
    kp2, des2 = orb.detectAndCompute(test_img, None)

    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

    if des1.any() or des2.any():
        matches = bf.match(des1, des2)
    else:
        return none

    # print "matches"
    # print(len(matches))

    return len(matches)


def draw_circles(img, circles):
    for i in circles[0, :]: # todo non null
        center = (i[0], i[1])
        cropped_circle_img = crop_circle_img(img, i, 30)
        radius = i[2]
        matches = sif_feature_detection(cropped_circle_img)
        if matches > MIN_MATCH_COUNT:
            cv.circle(img, center, radius, (255, 255, 255), 3)
        else:
            cv.circle(img, center, radius, (0, 0, 0), 3)
        # circle center
        # circle outline
    return img


def convert_to_robot_coordinates(coordinates):
    x_conversion = FOV_WIDTH / FOV_PIX_X
    y_conversion = FOV_HEIGHT / FOV_PIX_Y
    meter_coordinates = [0, 0]
    meter_coordinates[0] = 0.56367 - (coordinates[1] * y_conversion)
    meter_coordinates[1] = -0.2744 - (coordinates[0] * x_conversion)

    return meter_coordinates


def get_lfe_property(img):
    circles = detect_circles(img)
    if circles.any():
        circle = circles[0][0]
        cropped_circle_img = crop_circle_img(img, circle, 30)
        lfe_coordinate = LfeCoordinate()
        meter_coordinates = convert_to_robot_coordinates(circle)
        # print(sif_feature_detection(template, cropped_circle_img))
        if sif_feature_detection(cropped_circle_img) > MIN_MATCH_COUNT:
            lfe_coordinate.x_axe = meter_coordinates[0]
            lfe_coordinate.y_axe = meter_coordinates[1]
            lfe_coordinate.upside = True
        else:
            lfe_coordinate = LfeCoordinate()
            lfe_coordinate.x_axe = meter_coordinates[0]
            lfe_coordinate.y_axe = meter_coordinates[1]
            lfe_coordinate.upside = False
        return lfe_coordinate
    else:
        # print("Finished")
        return None
