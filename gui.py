import tkinter.font
from tkinter import Tk, Canvas, Frame, BOTH
from parameters import vals

# Written by me
def coloring(grid, ex):
    i = len(grid)
    j = len(grid[0])
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            ex.topographic_grid(i, j, grid[i, j], grid)


# All of the GuiGrid class is written by me
class GuiGrid(Frame):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.master.title("Lines")
        self.pack(fill=BOTH, expand=1)
        global canvas
        canvas = Canvas(self)
        canvas.create_line(10, 10, 9.0625 * (vals.grid_size + 4) + 10 , 10, 9.0625 * (vals.grid_size + 4)+ 10,
                           9.0625 * (vals.grid_size + 4) + 10, 10, 9.0625 * (vals.grid_size + 4) + 10 , 10, 10)
        x = 19.0625
        y = (vals.grid_size + 4)*9.0625 + 10
        for i in range(vals.grid_size + 4):
            canvas.create_line(x, 10, x, y)
            x += 9.0625
        i = 0
        y = 19.0625
        for i in range(vals.grid_size + 4):
            canvas.create_line(10, y, (4+vals.grid_size)*9.0625 + 10 , y)
            y += 9.0625
        canvas.pack(fill=BOTH, expand=1)


    def pos(self, pos, robot_type):
        if robot_type == 1:
            canvas.create_text(pos[0] * 9.0625 + 15, pos[1] * 9.0625 + 19, font="Bold", text="*", tags="pos_navigator",
                               fill='red')
        elif robot_type == 2:
            canvas.create_text(pos[0] * 9.0625 + 15, pos[1] * 9.0625 + 19, font="Bold", text="*", tags="pos_digger",
                               fill='blue')
        elif robot_type == 3:
            canvas.create_text(pos[0] * 9.0625 + 15, pos[1] * 9.0625 + 19, font="Bold", text="*", tags="pos_shoveler",
                               fill='brown')
        elif robot_type == 4:
            # Not a real type, indicates that the robots finished their scan
            canvas.create_text(pos[0] * 9.0625 + 15, pos[1] * 9.0625 + 19, font="Bold", text="*", tags="pos",
                               fill='black')
        canvas.pack(fill=BOTH, expand=1)
        canvas.delete()

    def clear_canvas_navigator(self):
        canvas.delete("pos_navigator")

    def clear_canvas_digger(self):
        canvas.delete("pos_digger")

    def clear_canvas_shoveler(self):
        canvas.delete("pos_shoveler")

    def clear_canvas_2(self):
        canvas.delete("knownGrid")

    def topographic_grid(self, x, y, alt, grid):

        if alt == 1:
            canvas.create_rectangle(x * 9.0625 + 15 - 5, y * 9.0625 + 15 - 5, x * 9.0625 + 15 + 5, y * 9.0625 + 15 + 5,
                                    fill="#44D827")
        if alt == 2:
            canvas.create_rectangle(x * 9.0625 + 15 - 5, y * 9.0625 + 15 - 5, x * 9.0625 + 15 + 5, y * 9.0625 + 15 + 5,
                                    fill="#39B321")
        if alt == 3:
            canvas.create_rectangle(x * 9.0625 + 15 - 5, y * 9.0625 + 15 - 5, x * 9.0625 + 15 + 5, y * 9.0625 + 15 + 5,
                                    fill="#2D8F1A")
        if alt == 4:
            canvas.create_rectangle(x * 9.0625 + 15 - 5, y * 9.0625 + 15 - 5, x * 9.0625 + 15 + 5, y * 9.0625 + 15 + 5,
                                    fill="#236E14")
        if alt == 5:
            canvas.create_rectangle(x * 9.0625 + 15 - 5, y * 9.0625 + 15 - 5, x * 9.0625 + 15 + 5, y * 9.0625 + 15 + 5,
                                    fill="#1F6212")
        if alt == 6:
            grid[x, y] = 5
            canvas.create_rectangle(x * 9.0625 + 15 - 5, y * 9.0625 + 15 - 5, x * 9.0625 + 15 + 5, y * 9.0625 + 15 + 5,
                                    fill="#1F6212")
        if alt == -5:
            canvas.create_rectangle(x * 9.0625 + 15 - 5, y * 9.0625 + 15 - 5, x * 9.0625 + 15 + 5, y * 9.0625 + 15 + 5,
                                    fill="#a17312")
        if alt == 0:
            canvas.create_rectangle(x * 9.0625 + 15 - 5, y * 9.0625 + 15 - 5, x * 9.0625 + 15 + 5, y * 9.0625 + 15 + 5,
                                    fill="#453305")

    def local_global_grid(self, knownGrid):
        Desired_font = tkinter.font.Font(family="Comic Sans MS",
                                         size=5,
                                         weight="bold")
        for i in range(len(knownGrid)):
            for j in range(len(knownGrid)):
                if knownGrid[i, j] != -2:
                    canvas.create_text(i * 9.0625 + 605 + 36.25, j * 9.0625 + 15, font=Desired_font,
                                       text=knownGrid[i, j],
                                       tags="knownGrid", )
                else:
                    pass