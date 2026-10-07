from typing import List
from consts import Direction, EXPANDED_CELL, SCREENSHOT_COST, MAP_TO_PLANNER
from helper import is_valid


class CellState:
    """Base class for all objects on the arena, such as cells, obstacles, etc"""

    def __init__(self, x, y, direction: Direction = Direction.NORTH, screenshot_id=-1, penalty=0):
        self.x = x
        self.y = y
        self.direction = direction
        # If screenshot_od != -1, the snapshot is taken at that position is for the obstacle with id = screenshot_id
        self.screenshot_id = screenshot_id
        self.penalty = penalty  # Penalty for the view point of taking picture

    def __hash__(self):
        return hash((self.x, self.y, self.direction))

    def __eq__(self, other):
        return isinstance(other, CellState) and (self.x, self.y, self.direction) == (other.x, other.y, other.direction)

    def cmp_position(self, x, y) -> bool:
        """Compare given (x,y) position with cell state's position

        Args:
            x (int): x coordinate
            y (int): y coordinate

        Returns:
            bool: True if same, False otherwise
        """
        return self.x == x and self.y == y

    def is_eq(self, x, y, direction):
        """Compare given x, y, direction with cell state's position and direction

        Args:
            x (int): x coordinate
            y (int): y coordinate
            direction (Direction): direction of cell

        Returns:
            bool: True if same, False otherwise
        """
        return self.x == x and self.y == y and self.direction == direction

    def __repr__(self):
        return "x: {}, y: {}, d: {}, screenshot: {}".format(self.x, self.y, self.direction, self.screenshot_id)

    def set_screenshot(self, screenshot_id):
        """Set screenshot id for cell

        Args:
            screenshot_id (int): screenshot id of cell
        """
        self.screenshot_id = screenshot_id

    def get_dict(self):
        """Returns a dictionary representation of the cell

        Returns:
            dict: {x,y,direction,screeshot_id}
        """
        return {'x': self.x, 'y': self.y, 'd': self.direction, 's': self.screenshot_id}


class Obstacle(CellState):
    """Obstacle class, inherited from CellState"""

    def __init__(self, x: int, y: int, direction: Direction, obstacle_id: int):
        super().__init__(x, y, direction)
        self.obstacle_id = obstacle_id

    def __eq__(self, other):
        """Checks if this obstacle is the same as input in terms of x, y, and direction

        Args:
            other (Obstacle): input obstacle to compare to

        Returns:
            bool: True if same, False otherwise
        """
        return self.x == other.x and self.y == other.y and self.direction == other.direction

    def get_view_state(self, retrying) -> List[CellState]:
        vectors = {Direction.NORTH: (0, 1), Direction.EAST: (1, 0),
                   Direction.SOUTH: (0, -1), Direction.WEST: (-1, 0)}
        if self.direction not in vectors:
            return []
        dx, dy = vectors[self.direction]
        side_x, side_y = (1, 0) if dx == 0 else (0, 1)
        facing = Direction((int(self.direction) + 4) % 8)
        choices = [(4, 0, 0), (5, 0, 0)] if retrying else [(3, 0, 5 * MAP_TO_PLANNER), (4, 0, 0)]
        choices += [(4, 1, SCREENSHOT_COST), (4, -1, SCREENSHOT_COST)]
        cells = []
        for distance, lateral, penalty in choices:
            x = self.x + (dx * distance + side_x * lateral) * MAP_TO_PLANNER
            y = self.y + (dy * distance + side_y * lateral) * MAP_TO_PLANNER
            if is_valid(x, y):
                cells.append(CellState(x, y, facing, self.obstacle_id, penalty))
        return cells


class Grid:
    """
    Grid object that contains the size of the grid and a list of obstacles
    """
    def __init__(self, size_x: int, size_y: int):
        """
        Args:
            size_x (int): Size of the grid in the x direction
            size_y (int): Size of the grid in the y direction
        """
        self.size_x = size_x
        self.size_y = size_y
        self.obstacles: List[Obstacle] = []

    def add_obstacle(self, obstacle: Obstacle):
        """Add a new obstacle to the Grid object, ignores if duplicate obstacle

        Args:
            obstacle (Obstacle): Obstacle to be added
        """
        # Loop through the existing obstacles to check for duplicates
        to_add = True
        for ob in self.obstacles:
            if ob == obstacle:
                to_add = False
                break

        if to_add:
            self.obstacles.append(obstacle)

    def reset_obstacles(self):
        """
        Resets the obstacles in the grid
        """
        self.obstacles = []

    def get_obstacles(self):
        """
        Returns the list of obstacles in the grid
        """
        return self.obstacles

    def reachable(self, x: int, y: int, turn=False, preTurn=False) -> bool:
        """Checks whether the given x,y coordinate is reachable/safe. Criterion is as such:
        - Must be at least 4 units away in total (x+y) from the obstacle
        - Greater distance (x or y distance) must be at least 3 units away from obstacle

        Args:
            x (int): _description_
            y (int): _description_

        Returns:
            bool: _description_
        """
        
        if not self.is_valid_coord(x, y):
            return False

        for ob in self.obstacles:
            dx, dy = abs(ob.x - x), abs(ob.y - y)
            if dx + dy >= 4 * MAP_TO_PLANNER:
                continue
            clearance = (5 if turn or preTurn else 3) * MAP_TO_PLANNER
            if max(dx, dy) < clearance:
                return False

        return True

    def is_valid_coord(self, x: int, y: int) -> bool:
        """Checks if given position is within bounds

        Args:
            x (int): x-coordinate
            y (int): y-coordinate

        Returns:
            bool: True if valid, False otherwise
        """
        if x < MAP_TO_PLANNER or x > self.size_x - 2 * MAP_TO_PLANNER or y < MAP_TO_PLANNER or y > self.size_y - 2 * MAP_TO_PLANNER:
            return False

        return True

    def is_valid_cell_state(self, state: CellState) -> bool:
        """Checks if given state is within bounds

        Args:
            state (CellState)

        Returns:
            bool: True if valid, False otherwise
        """
        return self.is_valid_coord(state.x, state.y)

    def get_view_obstacle_positions(self, retrying) -> List[List[CellState]]:
        """
        This function return a list of desired states for the robot to achieve based on the obstacle position and direction.
        The state is the position that the robot can see the image of the obstacle and is safe to reach without collision
        :return: [[CellState]]
        """
        # print(f"Inside get_view_obstacle_positions: retrying = {retrying}")
        optimal_positions = []
        for obstacle in self.obstacles:
            if obstacle.direction == 8:
                continue
            else:
                view_states = [view_state for view_state in obstacle.get_view_state(
                    retrying) if self.reachable(view_state.x, view_state.y)]
            optimal_positions.append(view_states)

        return optimal_positions