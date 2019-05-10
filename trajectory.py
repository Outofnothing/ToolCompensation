class Point:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

    def __repr__(self):
        return 'Point(%r, %r)'%(self.x, self.y)

    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        return Point(x, y)

    def __sub__(self, other):
        x = self.x - other.x
        y = self.y - other.y
        return Point(x, y)

class Line:
    def __init__(self, start,  goal):
        self.start = start
        self.goal = goal

class Arc:
    def __init__(self, radius, start, goal, origin, CLOCKWISE):
        self.start = start
        self.goal = goal
        self.radius = radius
        self.origin = origin
        self.CLOCKWISE = CLOCKWISE

def GeneralEquation(first_x,first_y,second_x,second_y):
    # 一般式 Ax+By+C=0
    # from http://www.cnblogs.com/DHUtoBUAA/
    A=second_y-first_y
    B=first_x-second_x
    C=second_x*first_y-first_x*second_y
    return A,B,C

def GetIntersectPointofLines(Line_1, Line_2):
    # from http://www.cnblogs.com/DHUtoBUAA/
    x1, y1 = Line_1.start.x, Line_1.start.y 
    x2, y2 = Line_1.goal.x, Line_1.goal.y 
    x3, y3 = Line_2.start.x, Line_2.start.y 
    x4, y4 = Line_2.goal.x, Line_2.goal.y 
    A1, B1, C1 = GeneralEquation(x1,y1,x2,y2)
    A2, B2, C2 = GeneralEquation(x3,y3,x4,y4)
    m = A1*B2-A2*B1
    if m==0:
        print("平行，无交点")
    else:
        x = (C2*B1-C1*B2)/m
        y = (C1*A2-C2*A1)/m
    return x, y

class Compensation:
    def __init__(self, radius, LEFT_CUT):
        self.radius = radius
        self.LEFT_CUT = LEFT_CUT
        self.line_list = []
        self.predifined_traj = []
        self.augmented_traj = []

    def line_cut(self, start, goal):
        diff = goal - start
        # 求得sin， cos
        sin_theta = diff.y/pow(pow(diff.x, 2) + pow(diff.y, 2), 0.5)
        cos_theta = diff.x/pow(pow(diff.x, 2) + pow(diff.y, 2), 0.5)
        
        if self.LEFT_CUT:
            # 平移直线
            comp_start = start + Point(- sin_theta*self.radius, cos_theta*self.radius)
            comp_goal = goal + Point(- sin_theta*self.radius, cos_theta*self.radius)
            
        else:
            comp_start = start + Point(+sin_theta*self.radius, -cos_theta*self.radius)
            comp_goal = goal + Point(+sin_theta*self.radius, -cos_theta*self.radius)
        self.line_list.append(Line(comp_start, comp_goal))
        print(comp_start, comp_goal)
        return comp_start, comp_goal
    
    def arc_cut(self, arc):
        assert(arc.radius < self.radius, "错误：输入的刀具直径大于加工圆弧的直径")
        diff_s = arc.start - arc.origin
        diff_g = arc.goal - arc.origin
        sin_theta_s = diff_s.y/pow(pow(diff_s.x, 2) + pow(diff_s.y, 2), 0.5)
        cos_theta_s = diff_s.x/pow(pow(diff_s.x, 2) + pow(diff_s.y, 2), 0.5)

        sin_theta_g = diff_g.y/pow(pow(diff_g.x, 2) + pow(diff_g.y, 2), 0.5)
        cos_theta_g = diff_g.x/pow(pow(diff_g.x, 2) + pow(diff_g.y, 2), 0.5)
        # 同或操作
        if (not arc.CLOCKWISE and not self.LEFT_CUT) or (arc.CLOCKWISE and self.LEFT_CUT):
            comp_start = arc.start + Point(cos_theta_s*self.radius, sin_theta_s*self.radius)
            comp_goal = arc.goal + Point(cos_theta_g*self.radius, sin_theta_g*self.radius)
            comp_radius = arc.radius+self.radius
        else:
            comp_start = arc.start + Point(cos_theta_s*self.radius, sin_theta_s*self.radius)
            comp_goal = arc.goal + Point(cos_theta_g*self.radius, sin_theta_g*self.radius)
            comp_radius = arc.radius - self.radius
        return Arc(comp_radius, comp_start, comp_goal, arc.origin, arc.CLOCKWISE)
        
    def join_lines(self):
        self.augmented_traj.append(self.line_list[0].start)
        for i  in range(len(self.line_list)-1):
            intersection = GetIntersectPointofLines(self.line_list[i], self.line_list[i+1])
            self.augmented_traj.append(intersection)
        self.augmented_traj.append(self.line_list[-1].goal)

comp = Compensation(10, True)
a = Point(100, 10)
b = Point(100, 100)
c = Point(200, 100)
comp.line_cut(a, b)
comp.line_cut(b, c)
comp.join_lines()
print(comp.augmented_traj)
