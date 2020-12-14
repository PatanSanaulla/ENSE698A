import cv2

def getCountoursVisionSensor(sensorImage):
    if len(sensorImage) != 0:
        sensorImage = np.reshape(sensorImage, np.append(res, 3))
        sensorImage = cv2.resize(sensorImage, (1000, 1000), interpolation=cv2.INTER_AREA)
        sensorImage = cv2.flip(sensorImage, 0)  # vertical flip
        sensorImage = cv2.cvtColor(sensorImage, cv2.COLOR_RGB2GRAY)  # gray scale
        sensorImage = cv2.GaussianBlur(sensorImage, (5, 5), 0)
        edges = cv2.Canny(sensorImage, 100, 200)
        ret, binImg = cv2.threshold(edges, 0, 255, 0)
        contours, hierarchy = cv2.findContours(binImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return (True, contours)
    print("Failed to get Vision Sensor Image")
    return (False, 0)

