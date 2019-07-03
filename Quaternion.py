

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    def inverse(self):
        coeff = 1/(self.x**2 + self.y**2 + self.z**2 + self.w**2)
        return Quaternion(-1*coeff*self.x, -1*coeff*self.y, -1*coeff*self.z, coeff*self.w)
    def multiplication(self, q1):
        #Takes the product of self and q1
        x = self.w*q1.x + self.x*q1.w - self.y*q1.z + self.z*q1.y
        y = self.w*q1.y + self.x*q1.z + self.y*q1.w - self.z*q1.x
        z = self.w*q1.z - self.x*q1.y + self.y*q1.x + self.z*q1.w
        w = self.w*q1.w - self.x*q1.x - self.y*q1.y -  self.z*q1.z
        return Quaternion(x, y, z, w)
