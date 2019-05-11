from math import hypot

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
        self.radius = abs(goal-origin)
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
    if m==0:
        print("平行，无交点")
    else:
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
        for i in range(len(self.traj_list)-1):
            if type(self.traj_list[i]) is Line and type(self.traj_list[i+1]) is Line:
                intersection = GetIntersectPointofLines(self.traj_list[i], self.traj_list[i+1])
                
                
            elif type(self.traj_list[i]) is Line and type(self.traj_list[i+1]) is Arc:
                inter1, inter2 = GetIntersectPointofArcAndLine(self.traj_list[i+1], self.traj_list[i])
                # 靠近直线终点的是我们需要的那个交点
                if abs(inter1-self.traj_list[i].goal) <= abs(inter2-self.traj_list[i].goal):
                    intersection = inter1
                else:
                    intersection = inter2
            
            elif type(self.traj_list[i]) is Arc and type(self.traj_list[i+1]) is Arc: 
                pass

            elif type(self.traj_list[i]) is Arc and type(self.traj_list[i+1]) is Line:
                inter1, inter2 = GetIntersectPointofArcAndLine(self.traj_list[i], self.traj_list[i+1])
                # 靠近圆弧终点的是我们需要的那个交点
                if abs(inter1-self.traj_list[i].goal) <= abs(inter2-self.traj_list[i].goal):
                    intersection = inter1
                else:
                    intersection = inter2
            self.tool_traj[i].goal = intersection
            self.tool_traj[i+1].start = intersection
        # 填补终点到起点
        self.tool_traj.append(Line(self.tool_traj[-1].goal, self.tool_traj[0].start))
comp = Compensation(5, True)
line_1 = Line(Point(0, 0), Point(0, 100))
line_2 = Line(Point(0, 100), Point(100, 100))
arc_1 = Arc(Point(100, 100), Point(200, 0), Point(100, 0), True)
line_3 = Line(Point(200, 0), Point(0, 0))
comp.predifined_traj = [line_1, line_2, arc_1, line_3]
comp.join_trajectory()
print(comp.tool_traj)
