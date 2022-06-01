#!/usr/bin/env python3

"""
    Revolute Robot - Geometry
    Created by: Andrew O'Shei

    Description:
        Geometry classes for calculating collision avoidance and plotting.
"""

import numpy as np
import math

''' Class for defining a point in 3D space, with helper functions '''
class Point3D():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.t = np.array([[self.x, self.y, self.z, 1]]).T

    ''' Test if point p is the same as this one '''
    def is_equal(self, p):
        if self.x == p.x:
            if self.y == p.y:
                if self.z == p.z:
                    return True
        return False

    ''' Get the distance from self to the point p '''
    def get_distance(self, p):
        d = math.sqrt((p.x - self.x)**2 + (p.y - self.y)**2 + (p.z - self.z)**2)
        return d

    ''' Test if a point is on the line segment ls '''
    def is_on_line(self, ls):
        if self.is_equal(ls.p1):
            return True
        elif self.is_equal(ls.p2):
            return True
        else:
            min_x = min(ls.p1.x, ls.p2.x)
            max_x = max(ls.p1.x, ls.p2.x)
            if self.x > min_x and self.x < max_x:
                min_y = min(ls.p1.y, ls.p2.y)
                max_y = max(ls.p1.y, ls.p2.y)
                if self.y > min_y and self.y < max_y:
                    return True
        return False

class Target3D(Point3D):
    def __init__(self, x, y, z, origin=None, limit=None, floor=None):
        super().__init__(x, y, z)
        if origin is not None and limit is not None:
            self.__get_target_limit(origin, limit)
        if floor is not None:
            if self.z < floor:
                self.z = floor
        self.t = np.array([[self.x, self.y, self.z, 1]]).T

    # Finds the closest possible target within limit from origin
    def __get_target_limit(self, origin, limit):
        check = LineSegment(origin, self)
        if check.get_length() > limit:
            if check.slope == None:
                self.x = origin.x
                self.y = origin.y + limit
            elif abs(check.slope) < 0.00001:
                # Handle horizontal
                if self.x > origin.x:
                    self.x = origin.x + limit
                else:
                    self.x = origin.x - limit
            else:
                dx = (limit / math.sqrt(1 + (check.slope**2)))
                dy = check.slope * dx
                if self.x > 0:
                    self.x = origin.x + dx
                    self.y = origin.y + dy
                else:
                    self.x = origin.x - dx
                    self.y = origin.y - dy
            # print("Adjusted Target: {}, {}".format(target.x, target.y))


class LineSegment3d():
    def __init__(self, p1, p2):
        if p1.is_equal(p2):
            raise ValueError("Cannot form line from identical points")
        self.p1 = p1
        self.p2 = p2
        self.slope = self.get_slope(self.p1, self.p2)

    def get_midpoint(self):
        x_mid = (self.p1.x + self.p2.x)/2
        y_mid = (self.p1.y + self.p2.y)/2
        return Point(x_mid, y_mid)

    ''' Returns the slope of a line '''
    def get_slope(self, p1, p2):
        if p1.x == p2.x:
            return 0.0
        elif p1.y == p2.y:
            return None
        else:
            return (p2.y - p1.y) / (p2.x - p1.x)

    def get_perpendicular_slope(self):
        slope = self.get_slope(self.p1, self.p2)
        if slope == 0.0:
            return None
        elif slope is None:
            return 0.0
        else:
            return -1 / slope

    def get_length(self):
        return math.sqrt((self.p2.x - self.p1.x)**2 + (self.p2.y - self.p1.y)**2)

    ''' Returns a line perpendicular with self, which intersects at mid_p '''
    def get_perpendicular_line(self, mid_p, length):
        perpendicular_slope = self.get_perpendicular_slope()
        if perpendicular_slope is None:
            x1 = mid_p.x - (length/2)
            x2 = mid_p.x + (length/2)
            y1 = y2 = mid_p.y
        else:
            k_plus = (length/2)/math.sqrt(1 + perpendicular_slope**2)
            k_minus = (length/2)/math.sqrt(1 + perpendicular_slope**2) * -1
            x2 = mid_p.x + k_plus
            y2 = mid_p.y + k_plus * perpendicular_slope
            x1 = mid_p.x + k_minus
            y1 = mid_p.y + k_minus * perpendicular_slope
        p1 = Point(x1, y1)
        p2 = Point(x2, y2)
        return LineSegment(p1, p2)

    def test_parallel(self, ls):
        if (self.p2.y - self.p1.y) == 0:
            if (ls.p2.y - ls.p1.y) == 0:
                return True
            else:
                return False
        else:
            # Handle vertical slope
            s_sl = self.slope if self.slope is not None else 0.0
            l_sl = ls.slope if ls.slope is not None else 0.0
            if abs(s_sl - l_sl) < 0.0001:
                return True
        return False

    ''' Find where self and the line ls intersect '''
    def get_intersection(self, ls):
        if not self.test_parallel(ls):
            try:
                dx1 = self.p2.x - self.p1.x
                dy1 = self.p2.y - self.p1.y
                dx2 = ls.p2.x - ls.p1.x
                dy2 = ls.p2.y - ls.p1.y
                denominator = (dy1 * dx2 - dx1 * dy2)
                if abs(denominator) < 0.0001:
                    t = 1
                else:
                    t = ((self.p1.x - ls.p1.x) * dy2 + (ls.p1.y - self.p1.y) * dx2)/ denominator
                x_intersect = self.p1.x + dx1 * t
                y_intersect = self.p1.y + dy1 * t
                return Point(x_intersect, y_intersect)
            except ZeroDivisionError:
                pass
        return None

    def get_point_at_distance(self, p1, distance):
        r = math.sqrt( distance**2 / ( 1 + self.slope**2 ) )
        pd_minus = Point(p1.x - r, p1.y - self.slope * r)
        pd_plus = Point(p1.x + r, p1.y + self.slope * r)
        if pd_minus.get_distance(self.p1) < pd_minus.get_distance(self.p2):
            return pd_minus
        else:
            return pd_plus
