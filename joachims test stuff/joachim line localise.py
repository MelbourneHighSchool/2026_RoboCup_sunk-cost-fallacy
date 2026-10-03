import numpy as np
import cv2, os
from time import perf_counter
from multiprocessing import Value, Array
from ctypes import c_uint8, c_int16, c_float
# TODO: Make this shit cooler with a cooler algorithm
# 1. Find x peak 1 (most points)
# 2. Find second x peak (mostly most points, somewhat how close to what x peak 1 predicts)
# 3. Revise guess of x peak 1 using x peak 2 (mostly how close to what x peak 2 predicts, somewhat most points)
# 4. I guess just check which is better

import numpy as np

BIN_WIDTH = 10
ERROR_RADIUS = 10
FIELD_L = 140
FIELD_W = 90
PREDICTION_GAIN = 1
def count_bands(array, bin_width, phase_shift):
    xs, counts = np.unique(np.round((array - phase_shift) / bin_width), return_counts = True)
    xs *= bin_width
    xs += phase_shift
    return (xs, counts)
def findCentre(pcloud):
    rayAngles = pcloud[:, 0]
    rayDists = pcloud[:, 1]
    rayXs, rayYs = np.cos(rayAngles) * rayDists, np.sin(rayAngles) * rayDists

    # Make bins that overlap (width is twice spacing) so no case where points are split between two regions happens
    bandsX0 = count_bands(rayXs, BIN_WIDTH, 0)
    bandsXh = count_bands(rayXs, BIN_WIDTH, BIN_WIDTH / 2)
    # Note: The order here is like [1,2,3,4,5,1.5,2.5,3.5,4.5,5.5]
    bandsX = (np.concatenate((bandsX0[0], bandsXh[0])), np.concatenate((bandsX0[1], bandsXh[1])))

    bandsY0 = count_bands(rayYs, BIN_WIDTH, 0)
    bandsYh = count_bands(rayYs, BIN_WIDTH, BIN_WIDTH / 2)
    bandsY = (np.concatenate((bandsY0[0], bandsYh[0])), np.concatenate((bandsY0[1], bandsYh[1])))

    # Find first peak
    maxXi = np.argmax(bandsX[1])  # Index of max count
    maxXbandStart = bandsX[0][maxXi] - BIN_WIDTH / 2  # Lower and upper bounds of the bins
    maxXbandEnd = bandsX[0][maxXi] + BIN_WIDTH / 2
    peakX1rays = rayXs[(maxXbandStart <= rayXs) & (rayXs <= maxXbandEnd)]  # Rays inside the peak band
    fenceStart, fenceEnd = np.percentile(peakX1rays, [25, 75])  # Average all values within Q1 and Q3 to get x-coord
    x1 = np.average(peakX1rays[(fenceStart <= peakX1rays) & (peakX1rays <= fenceEnd)])

    maxYi = np.argmax(bandsY[1])
    maxYbandStart = bandsY[0][maxYi] - BIN_WIDTH / 2
    maxYbandEnd = bandsY[0][maxYi] + BIN_WIDTH / 2
    peakY1rays = rayYs[(maxYbandStart <= rayYs) & (rayYs <= maxYbandEnd)]
    fenceStart, fenceEnd = np.percentile(peakY1rays, [25, 75])
    y1 = np.average(peakY1rays[(fenceStart <= peakY1rays) & (peakY1rays <= fenceEnd)])

    # Find second peak
    # Idea is to score remaining bands by
    #   Primarily how many points are in them
    #   Secondarily how close are they to what's predicted by the first peak (± field l or w)

    # Actl icbb
    bandsX[1][(x1 - ERROR_RADIUS <= bandsX[0]) & (bandsX[0] <= x1 + ERROR_RADIUS)] = 0  # Remove scattered points around peak
    bandsX[1][:] = bandsX[1] - PREDICTION_GAIN*(np.abs(np.abs(bandsX[0] - x1) - FIELD_W))
    maxXi = np.argmax(bandsX[1])  # Index of max count
    maxXbandStart = bandsX[0][maxXi] - BIN_WIDTH / 2  # Lower and upper bounds of the bins
    maxXbandEnd = bandsX[0][maxXi] + BIN_WIDTH / 2
    peakX1rays = rayXs[(maxXbandStart <= rayXs) & (rayXs <= maxXbandEnd)]  # Rays inside the peak band
    fenceStart, fenceEnd = np.percentile(peakX1rays, [25, 75])  # Average all values within Q1 and Q3 to get x-coord
    x2 = np.average(peakX1rays[(fenceStart <= peakX1rays) & (peakX1rays <= fenceEnd)])

    if np.isnan(x2):
        x2 = x1 + FIELD_W * np.sign(bandsX[0][maxXi] - x1)
        # print("Estimated width", "+" if np.sign(x2 - x1) > 0 else "-")

    bandsY[1][(y1 - ERROR_RADIUS <= bandsY[0]) & (bandsY[0] <= y1 + ERROR_RADIUS)] = 0  # Remove scattered points around peak
    bandsY[1][:] = bandsY[1] - PREDICTION_GAIN*(np.abs(np.abs(bandsY[0] - y1) - FIELD_L))
    maxYi = np.argmax(bandsY[1])  # Index of max count
    maxYbandStart = bandsY[0][maxYi] - BIN_WIDTH / 2  # Lower and upper bounds of the bins
    maxYbandEnd = bandsY[0][maxYi] + BIN_WIDTH / 2
    peakY1rays = rayYs[(maxYbandStart <= rayYs) & (rayYs <= maxYbandEnd)]  # Rays inside the peak band
    fenceStart, fenceEnd = np.percentile(peakY1rays, [25, 75])  # Average all values within Q1 and Q3 to get x-coord
    y2 = np.average(peakY1rays[(fenceStart <= peakY1rays) & (peakY1rays <= fenceEnd)])

    if np.isnan(y2):
        y2 = y1 + FIELD_L * np.sign(bandsY[0][maxYi] - y1)
        # print("Estimated width", "+" if np.sign(y2 - y1) > 0 else "-")
    
    xc = (x1 + x2) / 2
    yc = (y1 + y2) / 2
    # print(x1, x2, y1, y2)
    return np.array((xc, yc))
def distance_regression(dist_px):
    return -13562.2348/(dist_px - 361.39354) - 35.70338

kernel = np.ones((20,20))
for i, yaw in zip([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 1002],[-60,0,0,0,0,0,0,0,0,0]):
    t0 = perf_counter()
    try:
        frame = cv2.imread(f"joachims test stuff/render{i}.png")
        center = (311,336)
        frame_shape = frame.shape[:2]
        frame_size = frame_shape[0] * frame_shape[1]
    except AttributeError:
        print(f"render{i}.png is not real") 
        continue
    frame_canny = cv2.Canny(frame, 50,100)
    lines_mask_frame = np.zeros(frame_shape)
    line_bounds_v = Array(c_uint8, (0, 0, 200, 255, 60, 255))
    estimated_pos_v = Array(c_int16, (0, 0))  # x, y
    yaw_v = Value(c_float, yaw)
    dbg_points = Array(c_float, 720)
    lines_mask_mask = np.zeros((640,640), dtype=c_uint8)
    # cv2.blur(frame, (9, 9))
    cv2.cvtColor(frame, cv2.COLOR_RGB2HSV_FULL, frame)
    lines_mask_frame = cv2.inRange(frame, np.array(line_bounds_v[:3]), np.array(line_bounds_v[3:]))
    cv2.circle(lines_mask_mask, center, 432, 255, cv2.FILLED)
    cv2.circle(lines_mask_mask, center, 60, 0, cv2.FILLED)
    cv2.morphologyEx(frame_canny, cv2.MORPH_CLOSE, kernel, frame_canny)
    cv2.bitwise_and(frame_canny, lines_mask_mask, dst = frame_canny)
    cv2.bitwise_and(lines_mask_frame, frame_canny, dst = lines_mask_frame)

    max_radius = int(np.hypot(frame_shape[0], frame_shape[1]) * 0.7)  # Rough estimate
    polar_img = cv2.warpPolar(
        lines_mask_frame,
        dsize=(max_radius, 360), # 360 angle steps, max_radius distance resolution
        center=center,
        maxRadius=max_radius,
        flags=cv2.WARP_POLAR_LINEAR + cv2.INTER_NEAREST
    )

    pcloud = []
    has_hits = polar_img > 0
    first_hit_radii = np.argmax(has_hits, axis=1)

    yaw = yaw_v.value * np.pi / 32767
    lines_contour = np.zeros(shape=(360, 1, 2), dtype=np.int32)

    for i in range(360):
        if first_hit_radii[i] <= 0:
            dbg_points[2*i] = 0
            dbg_points[2*i+1] = 0
            continue
        angle = i*2*np.pi/360
        pcloud.append((angle - yaw, distance_regression(first_hit_radii[i])))
        dbg_points[2*i] = first_hit_radii[i] * np.cos(pcloud[-1][0])
        dbg_points[2*i+1] = first_hit_radii[i] * np.sin(pcloud[-1][1])
        lines_contour[i][0] = np.array((first_hit_radii[i] * np.cos(angle), first_hit_radii[i] * np.sin(angle)), dtype=np.int32)

    if not pcloud:
        estimated_pos_v = (-32767, -32767)
    else:
        field_center = findCentre(np.array(pcloud)).astype(int)
        estimated_pos_v[:] = field_center
    print(t0 - perf_counter())
    for i, r in zip(\
                np.stack([\
                np.cos((np.arange(360) + yaw)* np.pi / 180),\
                np.sin((np.arange(360) + yaw)* np.pi / 180)\
                ]).T, \
                first_hit_radii):
        cv2.circle(frame, center + (i * r).astype(int),2,(255,255,255))
    cv2.imshow("piss", cv2.cvtColor(frame, cv2.COLOR_HSV2RGB_FULL))
    
    cv2.imshow("pissy", lines_mask_frame)
    cv2.setMouseCallback("piss", lambda e, x, y, f, p : print(frame[y][x]) if e == cv2.EVENT_LBUTTONDOWN else None)
    #print(ord("e"))
    match cv2.waitKey(0):
        case -1:
            continue
        case 101:
            raise SystemExit
        case _:
            continue