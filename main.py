import math
import sys                       # 导入"system"模块，程序结尾有"sys.exit()"退出指令
from trajectory import *       
import pygame as pg
import pygame.gfxdraw
from pygame.math import Vector2
import matplotlib.pyplot as plt  # "matplotlib"2D绘图库 "pyplot"提供类似MATLAB的绘图框架
from matplotlib import animation # "animation"画动态图
from PyQt5 import uic, QtWidgets # "PyQt5"绘制界面

qtCreatorFile = "gui.ui"         # Enter file here.
DONE = True
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)
width, height = 1000, 1000
class Entity(pg.sprite.Sprite):

    def __init__(self, pos, traj,tool_radius, ani_speed):
        super().__init__()
        ATOM_IMG = pg.Surface((2*tool_radius, 2*tool_radius), pg.SRCALPHA)
        pg.gfxdraw.aacircle(ATOM_IMG, tool_radius, tool_radius, tool_radius, (255, 0, 0))
        pg.gfxdraw.filled_circle(ATOM_IMG, tool_radius, tool_radius, tool_radius, (255,0, 0))

        self.image = ATOM_IMG
        self.rect = self.image.get_rect(center=pos)
        self.vel = Vector2(0, 0)
        self.line_speed = ani_speed
        self.arc_speed = ani_speed/2
        self.traj = traj
        self.target = Vector2(self.traj.goal.get_tuple())      
        self.pos = Vector2(self.traj.start.get_tuple())
        self.done = False
        if type(traj) is Arc:
            self.origin = Vector2(traj.origin.get_tuple())
            self.radius_vector = self.pos- self.origin
    def update(self):
        if type(self.traj) is Line:
            # A vector pointing from self to the target.
            
            heading = self.target - self.pos
            distance = heading.length()  # Distance to the target.
            heading.normalize_ip()
            self.vel = heading * self.line_speed

            self.pos += self.vel
            # 使用左上角坐标系绘图
            self.rect.center = change_coor(self.pos)
            # print(distance)
            if distance <= 3:  # We're closer than 1 pixel.
                # 结束当前的轨迹描画，进入下一个轨迹
                self.done = True
        else:
            if self.traj.CLOCKWISE is True:
                angle = -self.arc_speed
            else:
                angle = self.arc_speed
            self.radius_vector = self.radius_vector.rotate(angle)
            # print(self.radius_vector)
            self.pos = self.radius_vector + self.origin
            self.rect.center = change_coor(self.pos)
            distance = (self.target - self.pos).length()
            if distance<=2:
                self.done = True

# 从标准坐标系转换到左上角坐标系
def change_coor(coord):
    return coord.x+width/2, height/2 - coord.y

class MyApp(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.predifined_traj = []
        self.add_line.clicked.connect(self.add_line_traj)
        self.add_arc.clicked.connect(self.add_arc_traj)
        self.add_arc_2.clicked.connect(self.add_arc_traj_IN)
        self.start.clicked.connect(self.draw)
        self.output_string = ""

    def add_line_traj(self):
        xg = float(self.xg_input.toPlainText())
        yg = float(self.yg_input.toPlainText())
        if len(self.predifined_traj) == 0:
            x0 = float(self.x0_input.toPlainText())
            y0 = float(self.y0_input.toPlainText())
            line = Line(Point(x0, y0), Point(xg, yg))
        else:
            line = Line(self.predifined_traj[-1].goal, Point(xg, yg))
        self.predifined_traj.append(line)
        self.output_string += "{0} added.\n".format(line)
        self.output.setText(self.output_string)
        print("{0} added.\n".format(line))

    def add_arc_traj(self):
        xg = float(self.xg_input.toPlainText())
        yg = float(self.yg_input.toPlainText())
        origin_x = float(self.origin_x.toPlainText())
        origin_y = float(self.origin_y.toPlainText())
        if len(self.predifined_traj) == 0:
            x0 = float(self.x0_input.toPlainText())
            y0 = float(self.y0_input.toPlainText())
            arc = Arc(Point(x0, y0), Point(xg, yg), Point(origin_x, origin_y), True)
        else:
            arc = Arc(self.predifined_traj[-1].goal, Point(xg, yg), Point(origin_x, origin_y), True)
        self.predifined_traj.append(arc)
        self.output_string += "{0} added.\n".format(arc)
        self.output.setText(self.output_string)
        print("{0} added.\n".format(arc))
    
    def add_arc_traj_IN(self):
        xg = float(self.xg_input.toPlainText())
        yg = float(self.yg_input.toPlainText())
        origin_x = float(self.origin_x.toPlainText())
        origin_y = float(self.origin_y.toPlainText())
        if len(self.predifined_traj) == 0:
            x0 = float(self.x0_input.toPlainText())
            y0 = float(self.y0_input.toPlainText())
            arc = Arc(Point(x0, y0), Point(xg, yg), Point(origin_x, origin_y), False)
        else:
            arc = Arc(self.predifined_traj[-1].goal, Point(xg, yg), Point(origin_x, origin_y), False)
        self.predifined_traj.append(arc)
        self.output_string += "{0} added.\n".format(arc)
        self.output.setText(self.output_string)
        print("{0} added.\n".format(arc))
    
    def draw(self):
        self.tool_radius = int(self.tool_radius_input.toPlainText())
        ani_speed = self.speed.value()/25 + 1 # from 1 to 5
        if self.tool_radius > 0:
            comp = Compensation(self.tool_radius, True)
        else:
            self.tool_radius = abs(self.tool_radius)
            comp = Compensation(self.tool_radius, False)
        comp.predifined_traj = self.predifined_traj
        self.output_string += "Predifined trajectory: {0}\n\n".format(self.predifined_traj)
        self.output.setText(self.output_string)
        comp.join_trajectory()
        screen = pg.display.set_mode((width, height))
        LINE_IMG = pg.Surface((width, height),pg.SRCALPHA)
        self.output_string += "Trajectory of the tool: {0}\n\n".format(comp.tool_traj)
        self.output.setText(self.output_string)
        print(comp.tool_traj)
        # 画出每条轨迹
        to_draw = []
        for traj in comp.tool_traj:
            to_draw.append(traj)
            clock = pg.time.Clock()
            entity = Entity(traj.start.get_tuple(), traj, self.tool_radius, ani_speed)
            all_sprites = pg.sprite.Group(entity)
            
            while not entity.done:
                
                for event in pg.event.get():
                    if event.type == pg.QUIT:
                        break
                
                all_sprites.update()
                screen.fill((30, 30, 30))
                # 画出预定义轨迹和走过的轨迹
                for route in comp.tool_traj:
                    start = change_coor(Vector2(route.start.get_tuple()))
                    end = change_coor(Vector2(route.goal.get_tuple()))
                    if type(route) is Line:
                        pg.draw.line(screen, (0, 0, 255), start, end)
                    else:
                        origin = change_coor(Vector2(route.origin.get_tuple()))
                        top_left = change_coor(Vector2(route.origin.x - route.radius, route.origin.y + route.radius))
                        
                        start_angle = Line(route.origin,route.start).angle()
                        end_angle = Line(route.origin, route.goal).angle()
                        # 如果是顺时针的则要反过来画圆弧
                        if route.CLOCKWISE is True:
                            end_angle, start_angle = start_angle, end_angle
                        pg.draw.arc(screen, (0, 0, 255), (top_left, (2*route.radius, 2*route.radius)), start_angle, end_angle)
                    
                for route in comp.predifined_traj:
                    start = change_coor(Vector2(route.start.get_tuple()))
                    end = change_coor(Vector2(route.goal.get_tuple()))
                    if type(route) is Line:
                        pg.draw.line(screen, (255, 255, 255), start, end)
                    else:
                        origin = change_coor(Vector2(route.origin.get_tuple()))
                        top_left = change_coor(Vector2(route.origin.x - route.radius, route.origin.y + route.radius))
                        
                        start_angle = Line(route.origin,route.start).angle()
                        end_angle = Line(route.origin, route.goal).angle()
                        # 如果是顺时针的则要反过来画圆弧
                        if route.CLOCKWISE is True:
                            end_angle, start_angle = start_angle, end_angle
                        pg.draw.arc(screen, (255, 255, 255), (top_left, (2*route.radius, 2*route.radius)), start_angle, end_angle)

                all_sprites.draw(screen)
                for point in [traj.start.get_tuple(), traj.goal.get_tuple()]:
                    point = change_coor(Vector2(point))
                    pg.draw.rect(screen, (90, 200, 40), (point, (4, 4)))

                pg.display.flip()
                clock.tick(30)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MyApp()
    window.show()
    sys.exit(app.exec_())
    