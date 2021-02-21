#! /usr/bin/python
# coding=UTF-8
import rospy
import sys
import numpy as np
from nav_msgs.msg import OccupancyGrid


class GridMap:
    def __init__(self, file_path):
        rospy.init_node('ORB_GRID_MAP')

        self.publisher = rospy.Publisher('/grid_map', OccupancyGrid, queue_size=10)
        self.free = -5.0
        self.occu = +15.0

        # m/cell
        self.resolution = 0.5

        self.keyframes, self.points, self.width, self.height = self.coordinateToGrid(
            *self.read_keyframe_points('temp.txt')
        )

        self.possibility_map = np.array([[50] * self.height] * self.width, dtype=np.int)
        print self.possibility_map.shape

    def read_keyframe_points(self, file_path):
        with open(file_path) as f:
            lines = f.read().splitlines()

        keyframes = []
        points = []

        i = 0
        while i < len(lines):
            if lines[i].find(' ') == -1:
                n = int(lines[i])
                keyframes.append([float(x) for x in lines[i+1].split(' ')])
                points.append([[float(y) for y in x.split(' ')] for x in lines[i+2: i+n+2]])
                i = i+n+2
            else:
                print 'wrong'
                break
        return keyframes, points

    def coordinateToGrid(self, _keyframes, _points):
        # 坐标是米，坐标/分辨率 就是grid坐标
        min_x, min_y, max_x, max_y = sys.maxint, sys.maxint, -sys.maxint, -sys.maxint
        keyframes = []
        points = []

        # 变换格式，计算包围方格
        for i in _keyframes:
            x, y = int(i[0] / self.resolution), int(i[2] / self.resolution)
            min_x, min_y, max_x, max_y = min(min_x, x), min(min_y, y), max(max_x, x), max(max_y, y)
            keyframes.append((x, y))

        for i in _points:
            tmp = []
            for j in i:
                x, y = int(j[0] / self.resolution), int(j[2] / self.resolution)
                min_x, min_y, max_x, max_y = min(min_x, x), min(min_y, y), max(max_x, x), max(max_y, y)
                tmp.append((x, y))
            points.append(tmp)

        return [(i[0]-min_x, i[1]-min_y) for i in keyframes], \
            [[(j[0]-min_x, j[1]-min_y) for j in i] for i in points], \
            max_x - min_x + 5, max_y - min_y + 5

    def get_line_bresenham(self, start, end):
        # https://zh.wikipedia.org/zh-cn/布雷森漢姆直線演算法

        if start == end:
            return [start]

        result = []
        x0, y0 = start
        x1, y1 = end
        steep = abs(y1 - y0) > abs(x1 - x0)

        if steep:
            x0, y0 = y0, x0
            x1, y1 = y1, x1
        if x0 > x1:
            x0, x1 = x1, x0
            y0, y1 = y1, y0

        deltax = x1 - x0
        deltay = abs(y1 - y0)
        error = 0.0
        deltaerr = deltay * 1.0 / deltax

        ystep = 0.0
        y = y0
        if y0 < y1:
            ystep = 1
        else:
            ystep = -1

        for x in range(x0, x1 + 1):
            if steep:
                if (y, x) != end:
                    result.append((y, x))
            else:
                if (x, y) != end:
                    result.append((x, y))

            error = error + deltaerr
            if error >= 0.5:
                y = y + ystep
                error = error - 1.0

        return result

    def iterate_update(self, keyframe, _points):
        for point in _points:
            result = self.get_line_bresenham(keyframe, point)
            self.possibility_map[point] = min(
                self.occu + self.possibility_map[point], 100
            )
            for i in result:
                self.possibility_map[i] = max(
                    self.free + self.possibility_map[i], 0
                )

    def ros_publish(self):
        r = rospy.Rate(10)
        
        msg = OccupancyGrid()
        msg.header.frame_id = 'grid'
        msg.header.stamp = rospy.Time.now()

        msg.info.resolution = self.resolution
        msg.info.width = self.height
        msg.info.height = self.width

        threshold = lambda x: 100 if x >= 70 else (0 if x <=5 else 50)
        msg.data = [threshold(x) for x in self.possibility_map.flatten()]

        while not rospy.is_shutdown():
            self.publisher.publish(msg)
            r.sleep()


def main():
    gridMap = GridMap('temp.txt')
    print len(gridMap.keyframes), len(gridMap.points)

    for i in range(len(gridMap.keyframes)):
        gridMap.iterate_update(gridMap.keyframes[i], gridMap.points[i])

    rospy.loginfo("===========")
    gridMap.ros_publish()


if __name__ == '__main__':
    main()
