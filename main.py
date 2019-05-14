import math
import sys                       # 导入"system"模块，程序结尾有"sys.exit()"退出指令
from trajectory import *       
import matplotlib.pyplot as plt  # "matplotlib"2D绘图库 "pyplot"提供类似MATLAB的绘图框架
from matplotlib import animation # "animation"画动态图
from PyQt5 import uic, QtWidgets # "PyQt5"绘制界面

qtCreatorFile = "gui.ui"         # Enter file here.

Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)


class MyApp(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.predifined_traj = []
        self.add_line.clicked.connect(self.add_line_traj)
        self.add_arc.clicked.connect(self.add_arc_traj)
        self.add_arc_2.clicked.connect(self.add_arc_traj_IN)


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
        print("{0} added.\n".format(arc))
    
    def draw_traj(self):
        pass


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MyApp()
    window.show()
    sys.exit(app.exec_())
