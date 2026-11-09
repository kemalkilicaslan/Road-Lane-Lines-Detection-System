# Road Lane Lines Detection System — Configuration

# Canny edge detection thresholds
CANNY_LOW = 100
CANNY_HIGH = 200

# Region of Interest (ROI) triangle — as ratios of frame (width, height): [x_ratio, y_ratio]
# e.g. [0.0, 1.0] = bottom-left corner, [0.5, 0.5] = frame center
ROI_VERTICES_RATIO = [[0.0, 1.0], [1.0, 1.0], [0.5, 0.5]]

# Hough transform parameters
HOUGH_THRESHOLD = 50
HOUGH_MIN_LINE_LENGTH = 50
HOUGH_MAX_LINE_GAP = 100

# Drawn lane line color (BGR) and thickness
LINE_COLOR = (0, 255, 0)
LINE_THICKNESS = 5

# Video files
INPUT_VIDEO  = "Road-Lane-Lines.mp4"           # input video file name
OUTPUT_VIDEO = "Detected-Road-Lane-Lines.mp4"  # output video file name
