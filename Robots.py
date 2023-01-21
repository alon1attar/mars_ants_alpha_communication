import random
import threading
import time
from abc import ABC
from collections import deque
from random import randint

import numpy
import numpy as np

from GUI import *
from grid_world import get_grid
from main import Environment
from utilities import grid_move_checker, direction_dict, Direction, graph_plot

global graph_mine_done
graph_mine_done = []
global graph_mine_found
graph_mine_found = []
global robot_list
robot_list = []
global starting_time
starting_time = time.time()


class RobotBase(ABC):
    def __init__(self, environment_obj, robot_id: int, grid_size=vals.grid_size):
        self.x = 0
        self.y = 0
        robot_list.append(self)
        self.simulation_time = []
        self.accuracy = []
        self.grid_size = grid_size
        self.grid = environment_obj.grid
        self.environment_obj = environment_obj
        self.robot_id = robot_id
        self.phase = 0
        self.mine_done = deque()
        self.view = vals.view
        mine_locations = np.where(self.grid == vals.mine_value)
        self.num_of_mines = mine_locations[0].shape[0]
        abs_loc = np.array(mine_locations) - np.tile(np.array(vals.starting_pos), (self.num_of_mines, 1)).T

    # Written by me
    def robots_communication_range(self,
                                   robot_type):  # needs to be implemented - not sure what are the rules for the communication
        InRangeRobotsList = []
        for i in range(len(robot_list)):
            pos1x = robot_list[i].x + vals.starting_pos[0]
            pos2x = self.get_pos_on_grid()[0]
            dis_x = abs(pos1x - pos2x)
            pos1y = robot_list[i].y + vals.starting_pos[1]
            pos2y = self.get_pos_on_grid()[1]
            dis_y = abs(pos1y - pos2y)
            if dis_x <= self.get_view_range(robot_type) // 2 and dis_y <= self.get_view_range(robot_type) // 2:
                InRangeRobotsList.append(robot_list[i])

        return InRangeRobotsList  # returns an array of the robots in range including self

    # Written by me
    def get_view_range(self, robot_type):
        return vals.communication_range

    # Lab code
    def get_pos_on_grid(self):
        x = np.clip(self.x + vals.starting_pos[0], 0, vals.grid_size - 1)
        y = np.clip(self.y + vals.starting_pos[1], 0, vals.grid_size - 1)
        return [x, y]

    # Written by me
    def update_accuracy_over_time(self, sim_time):
        self.simulation_time.append(sim_time)
        self.accuracy.append((np.where(self.knownGrid == self.grid)[0].shape[0]))

    # Lab code
    def get_view(self):
        x = self.get_pos_on_grid()[0]
        y = self.get_pos_on_grid()[1]
        grid = self.grid.copy()
        n = self.view // 2

        view = grid[x - n:x + n + 1, y - n:y + n + 1]

        return view


class Navigator(RobotBase):

    def __init__(self, environment_obj, robot_id):
        self.robot_type = 1
        super().__init__(environment_obj, robot_id)
        self.x = 0
        self.y = 0
        # relative to [0,0]:
        self.mine_found = deque()  # mine founds by the robots
        self.mine_done = deque()  # mine found and finished by the digger and shoveler
        self.mine_F_not_D = deque()  # mine found by the navigator and aren't done by the diggers (as far as the navigater knows)
        self.time_mine_done = []
        self.time_mine_found = []
        self.stamp = time.time()
        self.communication_load_navigator = 0

    # Lab code
    def one_step(self, direction, lock):

        if lock.acquire():
            # relative location

            self.x += direction_dict[direction]["change_x"]
            self.y += direction_dict[direction]["change_y"]
            lock.release()
        # actual location in grid

        self.environment_obj.set_robot_location(direction_dict[direction]["change_x"], self.robot_id, 0)
        self.environment_obj.set_robot_location(direction_dict[direction]["change_y"], self.robot_id, 1)

    # Lab code
    def find_next_mine_3(self, lock):
        direction1 = randint(0, 7)
        last_direction = direction1
        timeStatic = time.time()
        while (True):
            view = self.get_view()
            graph_mine_done.append(len(navigator_1.mine_done))
            graph_mine_found.append(len(navigator_1.mine_found))

            direction1 = randint(0, 7)

            mine_loc_rel = np.where(view == vals.mine_value)
            mines_in_view = mine_loc_rel[0].shape[0]
            if mines_in_view > 0:
                for k in range(len(view[0])):
                    for p in range(len(view[1])):
                        if view[k, p] == vals.mine_value:
                            RelDisY = -(vals.view // 2) + p
                            RelDisX = -(vals.view // 2) + k
                            x = self.x + RelDisX
                            y = self.y + RelDisY

                            if [x, y] not in self.mine_found:
                                print("Mine Location:", [x, y])
                                self.mine_found.append([x, y])
                                self.mine_F_not_D.append([x, y])
                                print(self.mine_found[-1])
                                self.time_mine_found.append(time.time() - self.stamp)
                                print(self.time_mine_found)
                                return

            if grid_move_checker(self.grid, self.environment_obj.robots_location[self.robot_id][0],
                                 self.environment_obj.robots_location[self.robot_id][1],
                                 self.environment_obj.robots_location[self.robot_id][0] +
                                 direction_dict[direction1]["change_x"],
                                 self.environment_obj.robots_location[self.robot_id][1] +
                                 direction_dict[direction1][
                                     "change_y"]):  # next step validation should be done using the view function
                self.one_step(direction1, lock)

    # Lab code
    def find_next_mine_2(self, lock):
        view = self.get_view()
        timeStatic = time.time()
        while (True):
            direction1 = randint(0, 7)
            mine_loc_rel = np.where(view == vals.mine_value)
            mines_in_view = mine_loc_rel[0].shape[0]
            mine_location_in_view = np.full((vals.view, vals.view), 0)

            if mines_in_view > 0:
                for k in range(len(view[0])):
                    for p in range(len(view[1])):
                        if view[k, p] == vals.mine_value:
                            RelDisY = -(vals.view // 2) + p
                            RelDisX = -(vals.view // 2) + k
                            x = self.get_pos_on_grid()[0] + RelDisX
                            y = self.get_pos_on_grid()[1] + RelDisY

                            if [x, y] not in self.mine_found:
                                print("Mine Location:", [x, y])
                                self.mine_found.append([x, y])
                                self.mine_F_not_D.append([x, y])
                                self.time_mine_found.append(time.time() - self.stamp)
                                return

            if grid_move_checker(self.grid, self.get_pos_on_grid()[0], self.get_pos_on_grid()[1],
                                 self.get_pos_on_grid()[0] + direction_dict[direction1]["change_x"],
                                 self.get_pos_on_grid()[1] + direction_dict[direction1][
                                     "change_y"]):  # next step validation should be done using the view function
                self.one_step(direction1, lock=lock)
                view = self.get_view()

            if time.time() >= timeStatic + 15:
                self.phase = 2
                break

    # Lab code
    def idle_state(self, robot):  # phase 4
        while robot.phase == 2:
            pass

        self.communicate_D_S()  # robot enter idle state when there are robots around him who are not open to communicate

    # Written by me
    def communicate_D_S(self):
        # ("commm")
        rangeRobots = self.robots_communication_range(self.robot_type)
        if len(rangeRobots) > 1:
            for robot in rangeRobots:
                if robot.robot_type == 3 or robot.robot_type == 2:
                    for place in self.mine_found:
                        if place not in robot.taskFound:
                            self.communication_load_navigator += 1
                            robot.taskLocationsQueue.append(place)
                            robot.taskFound.append(place)

    # Lab code
    def run_all_navigator(self, lock):
        self.phase = 1
        staticT = time.time()
        while len(self.mine_found) != self.num_of_mines:

            graph_mine_done.append(len(navigator_1.mine_done))
            graph_mine_found.append(len(navigator_1.mine_found))

            if self.phase == 1:
                if len(self.mine_found) != self.num_of_mines:
                    self.find_next_mine_3(
                        lock)
                    if len(self.robots_communication_range(
                            self.robot_type)) > 1:  # communication range includes the robot himself
                        self.phase = 3
                    else:
                        self.phase = 2
                else:
                    self.phase = 2
                graph_mine_done.append(len(navigator_1.mine_done))
                graph_mine_found.append(len(navigator_1.mine_found))

            if self.phase == 2:
                self.reach_last_mine(lock)
                if len(self.robots_communication_range(self.robot_type)) > 1:
                    self.phase = 3
                else:
                    self.phase = 2
            if self.phase == 3:
                # checks of robot is busy communicate otherwise
                InRange = self.robots_communication_range(self.robot_type)
                InRange.sort(key=lambda x: x.robot_type)
                for robot in InRange:
                    if robot.robot_type != 1:
                        if robot.phase == 3 and robot.robot_type != 1:
                            self.phase = 1
                        else:
                            self.communicate_D_S()
                self.phase = 1
        while len(self.mine_F_not_D) != 0:  # found all mines
            if len(self.robots_communication_range(self.robot_type)) > 1:
                InRange = self.robots_communication_range(self.robot_type)
                InRange.sort(key=lambda x: x.robot_type)
                for robot in InRange:
                    if (robot.phase != 3 and robot.robot_type != 1):  # the robots didn't finish all their tasks
                        # if robot.phase != 2: #robot not busy
                        self.communicate_D_S()
                    elif robot.robot_type != 1:
                        if len(self.mine_F_not_D) > 0:
                            self.reach_last_mine(lock)
                        else:
                            break
            else:
                self.reach_last_mine(lock)

        graph_finished = time.time() - starting_time
        graph_mine_found.insert(0, 0)
        graph_mine_done.append(self.num_of_mines)
        print(self.time_mine_done, "time mine done")
        print(self.time_mine_found, "time mine found")
        print("!!!!!!!!!!!!!!done!!!!!!!!!!!")
        graph_finished = time.time() - starting_time
        # End Of The Simulation Stats:
        print("Simulation Duration: ", graph_finished)
        print("Simulation Communication Load:",
        navigator_1.communication_load_navigator + digger_1.communication_load_digger)
        graph_plot(graph_finished, graph_mine_done, graph_mine_found)
        self.finished_bool = True

    # Lab code
    def reach_last_mine(self, lock):  # reach the last mine found to check of the digger and shoveler are there
        self.phase = 2  # communicate information

        # first mine found
        if len(self.mine_F_not_D) <= 1 and len(self.mine_done) == 0:
            mine = [0, 0]
        else:
            mine = self.mine_F_not_D[0]
        gap_x = abs(mine[0] - self.x)
        if (gap_x == 0):
            gap_x += 1
        for i in range(gap_x):
            for j in range(abs(mine[1] - self.y)):
                if mine[1] - self.y > 0:
                    self.one_step(Direction.R, lock)
                else:
                    self.one_step(Direction.L, lock)
            if (mine[0] - self.x < 0):
                self.one_step(Direction.T, lock)
            elif mine[0] - self.x > 0:
                self.one_step(Direction.D, lock)
        if (self.grid[
            self.environment_obj.robots_location[self.robot_id][0], self.environment_obj.robots_location[self.robot_id][
                1]] == vals.mine_val_finished):
            self.mine_done.append([self.x, self.y])
            self.mine_F_not_D.popleft()
            self.time_mine_done.append(time.time() - self.stamp)



# All of the Shoveler class is written by me

class Shoveler(RobotBase):
    def __init__(self, environment_obj, robot_id):
        self.robot_type = 3
        super().__init__(environment_obj, self.robot_type, robot_id)
        self.x = 0
        self.y = 0
        self.taskLocationsQueue = deque()
        self.shoveler_mine_done = 0
        self.taskFound = deque()
        self.count = 0

    def state_idle(self):
        if len(self.taskLocationsQueue) > 0:
            self.phase = 1
        else:
            pass

    def state_finish(self):
        pass

    def state_move_to_mine(self, x, y, lock):
        # Moves only one step at a time since the function will be called in a while loop
        if self.x - x > 0:
            if grid_move_checker(self.grid, self.get_pos_on_grid()[0], self.get_pos_on_grid()[1],
                                 self.get_pos_on_grid()[0] - 1, self.get_pos_on_grid()[1]):
                if lock.acquire():
                    self.x -= 1
                    lock.release()
                self.pos = [self.x, self.y]
        elif self.x - x < 0:
            if grid_move_checker(self.grid, self.get_pos_on_grid()[0], self.get_pos_on_grid()[1],
                                 self.get_pos_on_grid()[0] + 1, self.get_pos_on_grid()[1]):
                if lock.acquire():
                    self.x += 1
                    lock.release()
                self.pos = [self.x, self.y]
        elif self.y - y > 0:
            if grid_move_checker(self.grid, self.get_pos_on_grid()[0], self.get_pos_on_grid()[1],
                                 self.get_pos_on_grid()[0], self.get_pos_on_grid()[1] - 1):
                if lock.acquire():
                    self.y -= 1
                    lock.release()
                self.pos = [self.x, self.y]
        elif self.y - y < 0:
            if grid_move_checker(self.grid, self.get_pos_on_grid()[0], self.get_pos_on_grid()[1],
                                 self.get_pos_on_grid()[0], self.get_pos_on_grid()[1] + 1):
                if lock.acquire():
                    self.y += 1
                    lock.release()
                self.pos = [self.x, self.y]

        if self.y == y and self.x == x:
            self.phase = 2

    def state_shovel(self):
        taskTime = np.random.normal(vals.time_to_shovel, 0.3)
        tStatic = time.time()
        if self.grid[self.get_pos_on_grid()[0], self.get_pos_on_grid()[1]] == vals.mine_val_digger_finished:
            while time.time() < tStatic + taskTime:
                time.sleep(1)
                # pass
            else:
                # self.shoveler_mine_done += 1
                self.taskLocationsQueue.popleft()
                # Sets the value of the mine to 0 on the grid
                self.grid[self.x + vals.starting_pos[0], self.y + vals.starting_pos[1]] = 0
                self.phase = 0
                self.count += 1
        else:
            pass

    def run_all_shoveler(self, lock):
        tStatic = time.time()
        while len(navigator_1.mine_done) != self.num_of_mines:
            if lock.acquire():
                graph_mine_done.append(len(navigator_1.mine_done))
                graph_mine_found.append(len(navigator_1.mine_found))
                lock.release()
            tStatic = time.time()
            if self.count == self.num_of_mines:
                self.phase = 3
                self.state_finish()

            if self.phase == 0:
                self.state_idle()
            elif self.phase == 1:

                self.state_move_to_mine(self.taskLocationsQueue[0][0], self.taskLocationsQueue[0][1], lock=lock)
            elif self.phase == 2:
                self.state_shovel()


# All of the Digger class is written by me

class Digger(RobotBase):
    def __init__(self, environment_obj, robot_id):
        self.robot_type = 2
        super().__init__(environment_obj, self.robot_type, robot_id)
        self.x = 0
        self.y = 0
        self.taskLocationsQueue = deque()
        self.taskFound = deque()
        self.count = 0
        self.communication_load_digger = 0

    def state_idle(self):
        # Applied when the taskListLocations is empty
        if len(self.taskLocationsQueue) > 0:
            self.phase = 1
        else:
            pass

    def state_finish(self):
        pass

    def state_move_to_mine(self, x, y, lock):
        # Moves only one step at a time since the function will be called in a while loop
        if self.x - x > 0:
            if grid_move_checker(self.grid, self.get_pos_on_grid()[0], self.get_pos_on_grid()[1],
                                 self.get_pos_on_grid()[0] - 1, self.get_pos_on_grid()[1]):
                if lock.acquire():
                    self.x -= 1
                    lock.release()
                self.pos = [self.x, self.y]
        elif self.x - x < 0:
            if grid_move_checker(self.grid, self.get_pos_on_grid()[0], self.get_pos_on_grid()[1],
                                 self.get_pos_on_grid()[0] + 1, self.get_pos_on_grid()[1]):
                if lock.acquire():
                    self.x += 1
                    lock.release()
                self.pos = [self.x, self.y]
        elif self.y - y > 0:
            if grid_move_checker(self.grid, self.get_pos_on_grid()[0], self.get_pos_on_grid()[1],
                                 self.get_pos_on_grid()[0], self.get_pos_on_grid()[1] - 1):
                if lock.acquire():
                    self.y -= 1
                    lock.release()
                self.pos = [self.x, self.y]
        elif self.y - y < 0:
            if grid_move_checker(self.grid, self.get_pos_on_grid()[0], self.get_pos_on_grid()[1],
                                 self.get_pos_on_grid()[0], self.get_pos_on_grid()[1] + 1):
                if lock.acquire():
                    self.y += 1
                    lock.release()
                self.pos = [self.x, self.y]

        if self.y == y and self.x == x:
            self.phase = 2

    def state_dig(self):
        taskTime = np.random.normal(vals.time_to_dig, 0.3)
        tStatic = time.time()
        while time.time() < tStatic + taskTime:
            time.sleep(1)
        else:
            if time.time() >= tStatic + taskTime:
                self.communicate_finished_digging()
                if len(self.taskLocationsQueue) != 0:
                    self.taskLocationsQueue.popleft()
                    self.count += 1
                self.phase = 0

    def communicate_finished_digging(self):
        InRangeRobotList = self.robots_communication_range(self.robot_type)
        self.communication_load_digger += 1
        for j in range(len(robot_list)):
            if robot_list[j].robot_type == 3:
                robot_list[j].grid[self.get_pos_on_grid()[0], self.get_pos_on_grid()[1]] = vals.mine_val_digger_finished
        if len(InRangeRobotList) != 0:
            for i in range(len(InRangeRobotList)):
                if InRangeRobotList[i].robot_type == 3:
                    if self.taskLocationsQueue[-1] not in InRangeRobotList[i].taskFound:
                        InRangeRobotList[i].taskLocationsQueue.append(self.taskLocationsQueue[-1])
                        InRangeRobotList[i].taskFound.append(self.taskLocationsQueue[-1])
        else:
            pass

    def run_all_digger(self, lock):
        tStatic = time.time()
        while len(navigator_1.mine_done) != self.num_of_mines:
            graph_mine_done.append(len(navigator_1.mine_done))
            graph_mine_found.append(len(navigator_1.mine_found))
            if self.count == self.num_of_mines:
                self.phase = 3
                self.state_finish()
            if self.phase == 0:
                # print("idle")
                self.state_idle()
            elif self.phase == 1:
                self.state_move_to_mine(self.taskLocationsQueue[0][0], self.taskLocationsQueue[0][1], lock=lock)
            elif self.phase == 2:
                self.state_dig()


if __name__ == "__main__":
    seed = random.randint(0, 100)
    random.seed(seed)
    print(f"-----------------The current seed is {seed} -----------------------------")

    grid = get_grid(vals.grid_size, vals.pad_width, vals.N)
    numberOfRobots = 5
    finish = False
    robots_location = np.full((numberOfRobots, 2), vals.starting_pos)
    environment1 = Environment(grid, robots_location, numberOfRobots, finish)
    starting_time = time.time()

    shoveler_1 = Shoveler(environment1, 4)
    navigator_1 = Navigator(environment1, 0)
    digger_1 = Digger(environment1, 2)

    lock = threading.Lock()
    global ex, knownGrid1

    global root
    t1 = threading.Thread(target=navigator_1.run_all_navigator, args=[lock])
    t3 = threading.Thread(target=digger_1.run_all_digger, args=[lock])
    t5 = threading.Thread(target=shoveler_1.run_all_shoveler, args=[lock])

    t1.start()
    t3.start()
    t5.start()
