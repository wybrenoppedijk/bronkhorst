import numpy as np
import cv2 as cv

from src.bronkhorst.scripts.vision.manta import get_image

MIN_MATCH_COUNT = 20
template = cv.imread('.template/upside.jpeg', cv.IMREAD_COLOR)


# def load_camera():
#     cap = cv.VideoCapture(0)
#     ret, frame = cap.read()
#     return canny_edge(frame)


def detect_circles(img):

    img = cv.medianBlur(img, 5)
    rows = img.shape[0]
    circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, 1, rows / 8,
                              param1=90, param2=20,
                              minRadius=30, maxRadius=65)
    print(circles)
    if circles is not None:
        return np.uint16(np.around(circles))
    else:
        return None


def crop_circle_img(img, circle, margin):
    print(circle)
    x1 = circle[1] - circle[2] - margin
    x2 = circle[1] + circle[2] + margin
    y1 = circle[0] - circle[2] - margin
    y2 = circle[0] + circle[2] + margin
    cropped_img = img[x1:x2, y1:y2]
    return cropped_img


def sif_feature_detection(template, test_img):
    orb = cv.ORB_create()
    kp1, des1 = orb.detectAndCompute(template, None)
    kp2, des2 = orb.detectAndCompute(test_img, None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)

    flann = cv.FlannBasedMatcher(index_params, search_params)


    matches = flann.knnMatch(des1, des2, k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            good.append(m)

    return len(good)


def draw_circles(img, circles):
    for i in circles[0, :]: # todo non null
        center = (i[0], i[1])

        cropped_circle_img = crop_circle_img(img, i, 30)

        radius = i[2]

        print(sif_feature_detection(template, cropped_circle_img))

        if sif_feature_detection(template, cropped_circle_img) > MIN_MATCH_COUNT:
            cv.circle(img, center, radius, (255, 0, 0), 3)
            print("bovenkant")
        else:
            cv.circle(img, center, radius, (0, 255, 0), 3)
            print("onderkant")


        # circle center
        # circle outline
    return img


img = get_image()

circles = detect_circles(img)

cv.imshow('test', (draw_circles(img, circles)))
cv.waitKey(0)

# When everything done, release the capture
cv.destroyAllWindows()
