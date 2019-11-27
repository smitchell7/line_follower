#!/usr/bin/env python

import cv2 as cv
import numpy as np


def to_gray(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    return gray


def get_contours(frame):
    img, contours, _ = cv.findContours(frame, 1, 2)
    return (img, contours)


def draw_contours(frame, contours):
    g_center = 0.0
    img = frame
    for cnt in contours:
        M = cv.moments(cnt)
        if M['m00'] > 0.0:
            cX = int(M["m10"] / M["m00"])
            w = frame.shape[1]
            g_center += float(cX - w/2) / w
            img = cv.drawContours(img, [cnt], 0, (255, 0, 0), 8)
    return img, g_center


def crop(frame, xrange, yrange):
    h = frame.shape[0]
    w = frame.shape[1]
    xmin = int(xrange[0]*w)
    xmax = int(xrange[1]*w)
    ymin = int(yrange[0]*h)
    ymax = int(yrange[1]*h)
    img = frame[ymin:ymax, xmin:xmax]
    return img


def get_lines(frame):
    rho = 2  # distance resolution in pixels of the Hough grid
    theta = np.pi/180  # angular resolution in radians of the Hough grid
    # minimum number of votes (intersections in Hough grid cell)
    threshold = 15
    # minimum number of pixels making up a line
    min_line_length = 40
    # maximum gap in pixels between connectable line segments
    max_line_gap = 30
    lines = cv.HoughLinesP(frame, rho, theta, threshold, np.array([]),
                           min_line_length, max_line_gap)
    return lines


def draw_lines(frame, lines):
    line_image = np.copy(frame)*0  # creating a blank to draw lines on
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 5)
    img = cv.addWeighted(frame, 0.8, line_image, 1, 0)
    return img


def do_threshold(frame):
    frame = cv.medianBlur(frame, 9)
    # frame = cv.GaussianBlur(frame, (11,11), 0)
    img = cv.adaptiveThreshold(
        frame, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2)
    img = cv.bitwise_not(img)
    return img


def draw_moments(img):
    img, ctr, h = cv.findContours(img, 1, 2)
    for c in ctr:
        m = cv.moments(c)
        cx = int(m['m10'] / m['m00'])
        cy = int(m['m01'] / m['m00'])
        print(cx)
        print(cy)
    return img


def do_canny(frame):
    canny = cv.Canny(frame, 20, 100)
    return canny


def dilate_erode(frame):
    kernel = np.ones((5, 5), np.uint8)
    img = cv.morphologyEx(frame, cv.MORPH_CLOSE, kernel)
    return img


def get_line(frame):
    pass
