from math import hypot
import math

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

    def __abs__(self):
        return hypot(self.x, self.y)

    def __sub__(self, other):
        x = self.x - other.x
        y = self.y - other.y
        return Point(x, y)

    def __mul__(self, scalar):
        return Point(self.x*scalar, self.y*scalar) 

    def __truediv__(self, scalar):
        return Point(self.x/scalar, self.y/scalar)

    def T(self):
        return Point(-self.y, self.x)

class Line:
    def __init__(self, start,  goal):
        self.start = start
        self.goal = goal
    
    def __repr__(self):
        return "Line start at {0}, ends at {1}".format(self.start, self.goal)

class Arc:
    def __init__(self, start, goal, origin, CLOCKWISE):
        self.start = start
        self.goal = goal
        self.radius = float(abs(goal-origin))
        assert self.radius==abs(start - origin), "错误：圆弧参数错误，无法形成一个圆弧"
        self.origin = origin
        self.CLOCKWISE = CLOCKWISE

    def __repr__(self):
        return 'Arc starts at {0}, ends at {1}, origin at {2}, Clockwise:{3}'.format(self.start,
                                self.goal, self.origin, self.CLOCKWISE)

def GeneralEquation(line):
    # 一般式 Ax+By+C=0
    # from http://www.cnblogs.com/DHUtoBUAA/
    first_x,first_y = line.start.x, line.start.y
    second_x, second_y = line.goal.x, line.goal.y
    A=second_y-first_y
    B=first_x-second_x
    C=second_x*first_y-first_x*second_y
    return A,B,C

def GetIntersectPointofLines(Line_1, Line_2):
    # from http://www.cnblogs.com/DHUtoBUAA/
    A1, B1, C1 = GeneralEquation(Line_1)
    A2, B2, C2 = GeneralEquation(Line_2)
    m = A1*B2-A2*B1
    assert m!=0, "错误：两直线平行，无交点"
    x = (C2*B1-C1*B2)/m
    y = (C1*A2-C2*A1)/m
    return Point(x, y)

def GetIntersectPointofArcAndLine(arc, line):
    A, B, C = GeneralEquation(line)
    distance = abs(A*arc.origin.x + B*arc.origin.y+C)/pow(A**2+B**2, 0.5)
    assert distance<=arc.radius, "错误：直线与圆没有交点"
    delta = pow(arc.radius**2 - distance**2, 0.5)
    start = line.start
    goal = line.goal
    diff = goal - start
    # 求得sin， cos
    sin_theta = diff.y/pow(pow(diff.x, 2) + pow(diff.y, 2), 0.5)
    cos_theta = diff.x/pow(pow(diff.x, 2) + pow(diff.y, 2), 0.5)
    # 计算出圆心与直线垂足
    foot_x = (B*B*arc.origin.x - A*B*arc.origin.y - A*C)/(A**2 + B**2)
    foot_y = (A*A*arc.origin.y - A*B*arc.origin.x - B*C)/(A**2 + B**2)
    p = Point(foot_x, foot_y)
    inter1 = p + Point(cos_theta*delta, sin_theta*delta)
    inter2 = p - Point(cos_theta*delta, sin_theta*delta)
    return inter1, inter2

def GetIntersectPointofArcs(arc1, arc2):
    # 参见https://blog.csdn.net/zx3517288/article/details/53326420
    d = abs(arc1.origin - arc2.origin)
    assert d != 0, "错误，两个圆的圆心重合，无法计算交点"
    cos_theta = (arc1.radius**2+d**2-arc2.radius**2)/(2*arc1.radius*d)
    print(cos_theta)
    middle = (arc2.origin - arc1.origin)*cos_theta*arc1.radius/d + arc1.origin
    diff = arc2.origin - arc1.origin
    # 求得sin， cos
    sin_alpha = diff.y/pow(pow(diff.x, 2) + pow(diff.y, 2), 0.5)
    cos_alpha = diff.x/pow(pow(diff.x, 2) + pow(diff.y, 2), 0.5)
    theta = math.acos(cos_theta)
    sin_theta = math.sin(theta)
    h = sin_theta*arc1.radius
    print(h, d)
    print(middle)
    inter1 = middle + (arc2.origin - arc1.origin).T()*(h/d)
    inter2 = middle - (arc2.origin - arc1.origin).T()*(h/d)
    return inter1, inter2
    
class Compensation:
    def __init__(self, radius, LEFT_CUT):
        self.radius = radius
        self.LEFT_CUT = LEFT_CUT
        self.traj_list = []
        self.predifined_traj = []
        self.tool_traj = []

    # 求出直线轨迹的刀具中心轨迹
    def line_cut(self, line):
        start = line.start
        goal = line.goal
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
        
        self.traj_list.append(Line(comp_start, comp_goal))
        
    
    # 求出圆弧轨迹的刀具中心轨迹
    def arc_cut(self, arc):
        assert arc.radius > self.radius, "错误：输入的刀具直径大于加工圆弧的直径"
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

        self.traj_list.append(Arc(comp_start, comp_goal, arc.origin, arc.CLOCKWISE))

    # 将断续的轨迹连成完整的轨迹
    def join_trajectory(self):
        for traj in self.predifined_traj:
            if type(traj) == Line:
                self.line_cut(traj)
            else:
                self.arc_cut(traj)

        self.tool_traj= self.traj_list.copy()
        for i in range(len(self.traj_list)):
            current_traj = self.traj_list[i]
            if i == len(self.traj_list)-1:
                next_traj = self.traj_list[0]
            else:
                next_traj = self.traj_list[i+1]
            
            if type(current_traj) is Line and type(next_traj) is Line:
                intersection = GetIntersectPointofLines(current_traj, next_traj)
                
                
            elif type(current_traj) is Line and type(next_traj) is Arc:
                inter1, inter2 = GetIntersectPointofArcAndLine(next_traj, current_traj)
                # 靠近直线终点的是我们需要的那个交点
                if abs(inter1-current_traj.goal) <= abs(inter2-current_traj.goal):
                    intersection = inter1
                else:
                    intersection = inter2
            
            elif type(current_traj) is Arc and type(next_traj) is Arc: 
                inter1, inter2 = GetIntersectPointofArcs(current_traj, next_traj)
                # 靠近圆弧终点的是我们需要的那个交点
                if abs(inter1-current_traj.goal) <= abs(inter2-current_traj.goal):
                    intersection = inter1
                else:
                    intersection = inter2

            else: # type(current_traj) is Arc and type(next_traj) is Line:
                inter1, inter2 = GetIntersectPointofArcAndLine(current_traj, next_traj)
                # 靠近圆弧终点的是我们需要的那个交点
                if abs(inter1-current_traj.goal) <= abs(inter2-current_traj.goal):
                    intersection = inter1
                else:
                    intersection = inter2
            if i == len(self.traj_list)-1:
                self.tool_traj[i].goal = intersection
                self.tool_traj[0].start = intersection
            else:
                self.tool_traj[i].goal = intersection
                self.tool_traj[i+1].start = intersection

comp = Compensation(5, True)
arc_1 = Arc(Point(-200, 0), Point(100, 100), Point(100, -400), True)

arc_2 = Arc(Point(100, 100), Point(200, 0), Point(100, 0), True)
line_3 = Line(Point(200, 0), Point(-200, 0))
comp.predifined_traj = [arc_1, arc_2,line_3]
comp.join_trajectory()
print(comp.tool_traj)
