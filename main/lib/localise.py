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
        print("Estimated width", "+" if np.sign(x2 - x1) > 0 else "-")

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
        print("Estimated width", "+" if np.sign(y2 - y1) > 0 else "-")
    
    xc = (x1 + x2) / 2
    yc = (y1 + y2) / 2
    print(x1, x2, y1, y2)
    return np.array((xc, yc))