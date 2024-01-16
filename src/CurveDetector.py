import math

class CurveDetector:
    def __init__(self, maxCurveAngle, minCurveAngle, curveAngleTolerance, minCurveLength, maxCurveLength):
        self.maxCurveAngle = maxCurveAngle
        self.minCurveAngle = minCurveAngle
        self.curveAngleTolerance = curveAngleTolerance
        self.minCurveLength = minCurveLength
        self.maxCurveLength = maxCurveLength

    def detectCurves(self, points):
        """
        Detects curves in a list of points.
        :param points: The list of points to detect curves in.
        :return: A list of curves.
        """
        curves = []
        for i in range(len(points)):
            if i == 0:
                continue
            previousPoint = points[i - 1]
            currentPoint = points[i]
            curveAngle = self.__getCurveAngle(previousPoint, currentPoint)
            if curveAngle < self.minCurveAngle or curveAngle > self.maxCurveAngle:
                continue
            curve = self.__getCurve(points, i)
            if curve is not None:
                curves.append(curve)
        return curves

    def __getCurve(self, points, index):
        """
        Gets a curve from a list of points.
        :param points: The list of points to get the curve from.
        :param index: The index of the curve in the list of points.
        :return: A curve.
        """
        curve = []
        for i in range(index, len(points)):
            previousPoint = points[i - 1]
            currentPoint = points[i]
            curveAngle = self.__getCurveAngle(previousPoint, currentPoint)
            if curveAngle < self.minCurveAngle or curveAngle > self.maxCurveAngle:
                break
            curve.append(currentPoint)
        if len(curve) < self.minCurveLength or len(curve) > self.maxCurveLength:
            return None
        return curve

    def __getCurveAngle(self, previousPoint, currentPoint):
        """
        Gets the angle of a curve.
        :param previousPoint: The previous point.
        :param currentPoint: The current point.
        :return: The angle of the curve.
        """
        return math.degrees(math.atan2(currentPoint[1] - previousPoint[1], currentPoint[0] - previousPoint[0]))