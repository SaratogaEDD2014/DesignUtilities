import math
import wx
from numpy import *

def fequal(num1, num2, tolerance=.001):
    return abs(num1-num2)<tolerance

def Point3D(x,y,z):
    return (x,y,z)

class LineSegment():
    def __init__(self, x1, y1, x2, y2):
        self.set_endpoints(x1, y1, x2, y2)
    def get_slope(self):
        numerator = self.y2 - self.y1
        denom = self.x2 - self.x1
        if fequal(denom, 0):
            return float('inf')
        else:
            return numerator/denom
    slope = property(get_slope)

    def get_length(self):
        return math.sqrt((self.x1-self.x2)**2 + (self.y1-self.y2)**2)
    length = property(get_length)

    def calc_x(self, y, bound=True):
        """ Given a y value, it returns the points at the intersections of self and the line y=y"""
        y = float(y)
        m = self.get_slope()
        if not bound or y<=max(self.y1, self.y2) and y>=min(self.y2, self.y1):
            if m == float('inf'):
                #If vertical, and y is in range, we want the point that satisfies the x of the line and the given y
                return [(self.x1, y)]
            if m == 0:
                #If horizontal, to satisfy initial condition we know it is already equal, so return both endpoints
                if fequal(self.x1, self.x2):
                    #If both points are the same, just return one
                    return [(self.x1, y)]
                return [(self.x1, y), (self.x2, y)]
            #For normal points on a non-vertical, and non-horizontal line, simply use point slope calculation
            x = ((y-self.y1)/m)+self.x1
            return [(x,y)]
        else:
            return [] #No intersection

    def calc_y(self, x, bound=True):
        """ Given a x value, it returns the points at the intersections of self and the line x=x"""
        x = float(x)
        m = self.get_slope()
        if not bound or x<=max(self.x1, self.x2) and x>=min(self.x1, self.x2):
            if fequal(m, 0.0):
                #If line is horizontal, and x is in domain, we want only the point at x with the y of the line
                return [(x, self.y1)]
            if m == float('inf'):
                #The line is vertical, and x has already been confirmed to be in domain, so is equal to x of line
                if fequal(self.y1, self.y2):
                    #If marked as vertical because it is defined by two of the same points, return just one copy of the point
                    return [(self.x1, self.y1)]
                else:
                    #REturn the endpoints, because on this non-function segment their are two acceptable y-values
                    return [(self.x1,self.y1), (self.x2,self.y2)]
            #For normal points on a non-vertical, and non-horizontal line, simply use point slope calculation
            y = (m*(x-self.x1))+self.y1
            return [(x,y)]
        else:
            return [] #There is no intersection

    def m_x_b(self):
        b = self.calc_y(0, bound=False)
        if len(b) == 0:
            #only occurs with vertical line that does not intersect, so give x
            b = [calc_x(0)]
        return self.slope, b[0][1]
    def get_endpoints(self):
        return [(self.x1, self.y1), (self.x2, self.y2)]
    def set_endpoints(self, x1, y1, x2, y2):
        self.x1 = float(x1)
        self.y1 = float(y1)
        self.x2 = float(x2)
        self.y2 = float(y2)

    endpoints = property(get_endpoints, set_endpoints)

    def intersection(self, other):
        m1,b1 = self.m_x_b()
        m2,b2 = other.m_x_b()
        if fequal(m1, m2):
            #parallel
            if fequal(b1,b2):
                #collinear-give them point at zero
                return self.calc_y(x, bound=False)[0]
            else:
                #never touch
                return None
        else:
            x = (b1-b2)/(m2-m1)
            return self.calc_y(x, bound=False)[0]

class Line3d():
    def __init__(self, p1, p2):
        self.x1 = p1[0]
        self.y1 = p1[1]
        self.z1 = p1[2]
        self.x2 = p2[0]
        self.y2 = p2[1]
        self.z2 = p2[2]

    def calc_xy(self, z):
        z = float(z)
        line_xz = LineSegment(self.x1, self.z1, self.x2, self.z2) #Define line with X as independent, Z as dependent
        x_list = line_xz.calc_x(z)                                #Calculate the specific X for the given Z
        if len(x_list)<1:
            #does not cross this z value
            return []
        else:
            points = []
            x = x_list[0][0]
            line_xy = LineSegment(self.x1, self.y1, self.x2, self.y2)
            y = line_xy.calc_y(x)
            if len(y)<1:
                #Is not part of the domain
                return []
            points.append((x, y[0][1]))
            if len(y)>1:
                points.append((x, y[1][1]))

            if len(x_list)>1:
                x = x_list[1][0]
                y = line_xy.calc_y(x)
                if len(y)<1:
                    #Is not part of the domain
                    return []
                points.append((x, y[0][1]))
                if len(y)>1:
                    points.append((x, y[1][1]))
            return points

def line_from_angle(theta, r=20, off_x=0, off_y=0):
    return LineSegment(off_x, off_y, (r*math.cos(theta))+off_x, (r*math.sin(theta))+off_y)
def mirror_from_angle(theta, r=20, off_x=0, off_y=0):
    return Mirror(off_x, off_y, r*math.cos(theta)+off_x, r*math.sin(theta)+off_y)
def angle_from_line(segment):
    a = arctan(segment.slope)
    if a<0:
        a = 2*math.pi + a
    return a

class Mirror(LineSegment):
    def reflect(self, line, cut=True):
        intersect = self.intersection(line)
        incoming_theta = angle_from_line(line)
        if self.slope > 0:
            incoming_theta +=math.pi
        norm_theta = self.get_angle()+(math.pi/2)
        norm_theta -= int(norm_theta/(2*math.pi)) * (2*math.pi)
        if norm_theta < incoming_theta:
            new_theta = norm_theta - (incoming_theta-norm_theta)
        elif norm_theta > incoming_theta:
            new_theta = norm_theta + (norm_theta-incoming_theta)
        else:
            return line
        if cut:
            if self.slope < 0:
                if all (array([line.x1, line.y1]) <= array(intersect)):
                    #point one is low
                    line.x1, line.y1 = intersect
                else:
                    #point two is lower
                    line.x2, line.y2 = intersect
            if self.slope > 0:
                #mirror is increasing
                if line.x1 >= intersect[0] and line.y1<=intersect[1]:
                    #point one is lower
                    line.x1, line.x1 = intersect
                else:
                    #point two is lower
                    line.x2, line.y2 = intersect
            else:
                line.x2, line.y2 = intersect

        return line_from_angle(new_theta, off_x=intersect[0], off_y=intersect[1])
    def get_angle(self):
        return angle_from_line(self)



class MirrorDraw(wx.Frame):
    def __init__(self, limits=(10,15), pix_per_unit=40):
        super(MirrorDraw, self).__init__(None)
        self.ppu = pix_per_unit
        self.limits = limits
        self.x_buff = 20
        self.y_buff = 55 #35 to accomodate frame border, 10 for label
        self.SetSize((self.x_buff+limits[0]*self.ppu, self.y_buff+limits[1]*self.ppu))
        self.SetBackgroundColour(wx.Colour(255,255,255))
        self.dc = wx.ClientDC(self)
        self.dc.SetAxisOrientation(True, True)
        self.dc.SetDeviceOrigin(self.x_buff, self.GetSize()[1]-self.y_buff)
        self.Show()
        self.draw_axes()

    def draw_axes(self):
        self.dc.DrawLine(0, -1*self.y_buff, 0, self.limits[1]*self.ppu)
        for h in range(0, self.limits[0],  self.limits[0]/5):
            self.dc.DrawText(str(h), h*self.ppu, 0)
        for v in range(0, self.limits[1], self.limits[0]/5):
            self.dc.DrawText(str(v), self.x_buff-35 ,v*self.ppu)
        self.dc.DrawLine(-1*self.x_buff, 0, self.limits[1]*self.ppu, 0)

    def draw_segment(self, seg):
        points = array(seg.endpoints)*self.ppu
        self.dc.DrawLinePoint(points[0], points[1])

app = wx.App()
results = {}


#Old test results: best anlges: 313.7 and 44.2
#good new ones: 54-55:35-34
for test1 in [45]:#arange(48, 52, .1):
    for test in [45]:#arange(28, 32, .1):
        PROJ_ANGLE = .6057697
        PROJ_ORIGIN = (4-1.75,3)
        pane = MirrorDraw(limits=(14,14))
        pane.SetTitle(str(test1)+'/'+str(test))
        box = array([(0,0,   0,12),
                      (0,12, 4,12),
                      (4,12, 4,4.5),
                      (4, 4.5, 13,4.5),
                      (13,4.5, 13,0),
                      (13,0, 0,0)])*pane.ppu
        pane.dc.DrawLineList(box)

        m1_h = -5.5
        m1_a = test1
        m1_a = (180-m1_a)*math.pi/180 #360-46.3
        m1 = mirror_from_angle(m1_a, r=m1_h, off_y=abs(m1_h*math.sin(m1_a)))
        m2_h = 7.2
        m2_a = test*math.pi/180#44.2*math.pi/180
        m2 = mirror_from_angle(m2_a, r=m2_h, off_x=pane.limits[1]-abs(m2_h*math.cos(m2_a))-3)

        stopper = mirror_from_angle(0, 100, off_y=4.5)

        #Initial light path
        path1 = [line_from_angle((3*math.pi/2)-PROJ_ANGLE/2, off_x=PROJ_ORIGIN[0], off_y=PROJ_ORIGIN[1])]
        path2 = [line_from_angle((3*math.pi/2)+PROJ_ANGLE/2, off_x=PROJ_ORIGIN[0], off_y=PROJ_ORIGIN[1])]
        #After first mirror
        path1.append(m1.reflect(path1[0]))
        path2.append(m1.reflect(path2[0]))
        #After second mirror
        path1.append(m2.reflect(path1[1]))
        path2.append(m2.reflect(path2[1]))
        #Stop at pool-WILL NOT BE DRAWN
        stopper.reflect(path1[2])
        stopper.reflect(path2[2])

        #Mirrors
        pane.dc.SetPen(wx.Pen(wx.Colour(255,100,100), 3))
        pane.draw_segment(m1)
        pane.draw_segment(m2)

        #Projector
        pane.dc.SetPen(wx.Pen(wx.Colour(10,10,10), 2))
        pane.draw_segment(path1[0])
        pane.draw_segment(path2[0])

        #First Reflect
        pane.dc.SetPen(wx.Pen(wx.Colour(100,100,255), 2))
        pane.draw_segment(path1[1])
        pane.draw_segment(path2[1])

        #Second Reflect
        pane.dc.SetPen(wx.Pen(wx.Colour(100,255,100), 2))
        pane.draw_segment(path1[2])
        pane.draw_segment(path2[2])

        sum1 = path1[0].length + path1[1].length + path1[2].length
        sum2 = path2[0].length + path2[1].length + path2[2].length
        results[str(test1)+'/'+str(test)] = abs(sum1-sum2)

##        print test
##        print 'Path 1:', sum1
##        print 'Path 2:', sum2
##        print

        #pane.Close()


t = 10
best = ''
for key in results:
    if results[key]<t:
        t = results[key]
        best = key
        print "Combo ", key, " has New Best: ",t
    elif results[key]<.000001:
        print "Combo ", key, " was close: ",t
print 'Best: ', best
print 'With ', t
#print results

app.MainLoop()
