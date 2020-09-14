import math

class Vec2d:
    def __init__(self, x_=0.0, y_=0.0):
        self.x = x_
        self.y = y_

    def __mul__(self, v):
        #type:(Vec2d)->Vec2d
        return Vec2d(self.x*v.x,self.y*v.y)

    def __mul__(self, v):
        #type:(float)->Vec2d
        return Vec2d(self.x*v, self.y*v)

    def __truediv__(self, scale):
        return Vec2d(self.x/scale, self.y/scale)
    def cross(self,v):
        return self.x*v.y-self.y*v.x

    def dot(self, v):
        return self.x*v.x+self.y*v.y

    def __add__(self, v):
        return Vec2d(self.x+v.x, self.y+v.y)

    def __sub__(self, v):
        return Vec2d(self.x-v.x, self.y-v.y)
    def length(self):
        return math.sqrt(self.x**2+self.y**2)
    def norm(self):
        l = self.length()
        if math.fabs(l-0.0) < 1e-16:
            return Vec2d(0, 0)
        return Vec2d(self.x/l, self.y/l)