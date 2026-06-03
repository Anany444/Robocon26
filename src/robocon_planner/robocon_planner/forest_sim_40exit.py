import sys
import os
import copy
import random
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QComboBox
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush

class ForestSimApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robocon Forest Navigation Simulator")
        self.setGeometry(100, 100, 1200, 600)
        
        self.kfs_state = {i: "none" for i in range(1, 13)}
        self.start_kfs_state = {i: "none" for i in range(1, 13)}
        self.known_maze = {i: "unknown" for i in range(1, 13)}
        self.robot_pos = (4, 1) # Start at block 2 approach
        self.kfs_count = 0
        self.running = False
        self.log_msgs = []
        self.path_history = [(4, 1)]
        
        self.BLOCKS = {
            10: (0, 0), 11: (0, 1), 12: (0, 2),
            7:  (1, 0), 8:  (1, 1), 9:  (1, 2),
            4:  (2, 0), 5:  (2, 1), 6:  (2, 2),
            1:  (3, 0), 2:  (3, 1), 3:  (3, 2)
        }
        self.COORDS_TO_BLOCK = {v: k for k, v in self.BLOCKS.items()}
        
        self.initUI()
        
    def initUI(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout()
        main_widget.setLayout(layout)
        
        # Left side - Setup Grid
        left_layout = QVBoxLayout()
        left_layout.addWidget(QLabel("<b>Starting Configuration</b>"))
        self.grid_widget_left = QWidget()
        self.grid_widget_left.setFixedSize(350, 450)
        left_layout.addWidget(self.grid_widget_left)
        
        self.grid_layout = QGridLayout(self.grid_widget_left)
        self.combos = {}
        self.frames = {}
        for block_id, (r, c) in self.BLOCKS.items():
            box = QVBoxLayout()
            label = QLabel(f"Block {block_id}")
            label.setAlignment(Qt.AlignCenter)
            combo = QComboBox()
            combo.addItems(["none", "r1", "r2_real", "r2_fake"])
            combo.currentTextChanged.connect(lambda text, b=block_id: self.update_kfs(b, text))
            box.addWidget(label)
            box.addWidget(combo)
            
            frame = QWidget()
            frame.setLayout(box)
            self.grid_layout.addWidget(frame, r, c)
            self.combos[block_id] = combo
            self.frames[block_id] = frame
            
        layout.addLayout(left_layout)
        
        # Middle side - Dynamic Grid
        middle_layout = QVBoxLayout()
        middle_layout.addWidget(QLabel("<b>Live Simulation</b>"))
        self.grid_widget_right = QWidget()
        self.grid_widget_right.setFixedSize(350, 450)
        middle_layout.addWidget(self.grid_widget_right)
        layout.addLayout(middle_layout)
        
        # Right side - Controls and Logs
        right_layout = QVBoxLayout()
        
        controls = QHBoxLayout()
        self.team = "Red"
        self.team_btn = QPushButton("Team: Red")
        self.team_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.team_btn.clicked.connect(self.toggle_team)
        
        self.start_btn = QPushButton("Start Simulation")
        self.start_btn.clicked.connect(self.start_sim)
        self.reset_btn = QPushButton("Reset")
        self.reset_btn.clicked.connect(self.reset_sim)
        self.random_btn = QPushButton("Randomize")
        self.random_btn.clicked.connect(self.randomize_board)
        
        controls.addWidget(self.team_btn)
        controls.addWidget(self.start_btn)
        controls.addWidget(self.reset_btn)
        controls.addWidget(self.random_btn)
        right_layout.addLayout(controls)
        
        self.status_lbl = QLabel("Status: Setup (KFS: 0)")
        self.status_lbl.setStyleSheet("font-weight: bold; font-size: 14px;")
        right_layout.addWidget(self.status_lbl)
        
        self.log_lbl = QLabel("Logs:\n")
        self.log_lbl.setAlignment(Qt.AlignTop)
        self.log_lbl.setStyleSheet("background-color: black; color: lime; padding: 5px;")
        right_layout.addWidget(self.log_lbl)
        
        layout.addLayout(right_layout)
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.sim_step)
        
        # Setup Paint events
        self.grid_widget_left.paintEvent = lambda event: self.paint_grid(self.grid_widget_left, event, self.start_kfs_state, is_live=False)
        self.grid_widget_right.paintEvent = lambda event: self.paint_grid(self.grid_widget_right, event, self.kfs_state, is_live=True)
        
    def log(self, msg):
        self.log_msgs.append(msg)
        if len(self.log_msgs) > 20:
            self.log_msgs.pop(0)
        self.log_lbl.setText("Logs:\n" + "\n".join(self.log_msgs))
        print(msg)
        
    def toggle_team(self):
        if self.team == "Red":
            self.team = "Blue"
            self.team_btn.setText("Team: Blue")
            self.team_btn.setStyleSheet("background-color: blue; color: white; font-weight: bold;")
            self.BLOCKS = {
                12: (0, 0), 11: (0, 1), 10: (0, 2),
                9:  (1, 0), 8:  (1, 1), 7:  (1, 2),
                6:  (2, 0), 5:  (2, 1), 4:  (2, 2),
                3:  (3, 0), 2:  (3, 1), 1:  (3, 2)
            }
        else:
            self.team = "Red"
            self.team_btn.setText("Team: Red")
            self.team_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
            self.BLOCKS = {
                10: (0, 0), 11: (0, 1), 12: (0, 2),
                7:  (1, 0), 8:  (1, 1), 9:  (1, 2),
                4:  (2, 0), 5:  (2, 1), 6:  (2, 2),
                1:  (3, 0), 2:  (3, 1), 3:  (3, 2)
            }
            
        self.COORDS_TO_BLOCK = {pos: bid for bid, pos in self.BLOCKS.items()}
        
        for block_id, frame in self.frames.items():
            r, c = self.BLOCKS[block_id]
            self.grid_layout.removeWidget(frame)
            self.grid_layout.addWidget(frame, r, c)
            
        self.log(f"Switched to Team {self.team}")
        self.grid_widget_left.update()
        self.grid_widget_right.update()
        
    def randomize_board(self):
        import random
        
        while True:
            pool = ["r2_fake"]*1 + ["r1"]*3 + ["r2_real"]*4 + ["none"]*4
            random.shuffle(pool)
            config = {i: pool[i-1] for i in range(1, 13)}
            
            # Rule 1: r2_fake cannot be on blocks 1, 2, 3
            if config[1] == "r2_fake" or config[2] == "r2_fake" or config[3] == "r2_fake":
                continue
                
            # Rule 2: r1 can only be on boundary blocks, never on 5 or 8
            if config[5] == "r1" or config[8] == "r1":
                continue
                
            break
        
        for block_id in range(1, 13):
            val = config[block_id]
            self.kfs_state[block_id] = val
            self.start_kfs_state[block_id] = val
            self.combos[block_id].setCurrentText(val)
            
        self.grid_widget_left.update()
        self.grid_widget_right.update()
        self.log("Board randomized according to rules.")

    def update_kfs(self, block_id, text):
        if not self.running:
            self.start_kfs_state[block_id] = text
        self.kfs_state[block_id] = text
        self.grid_widget_left.update()
        self.grid_widget_right.update()
        
    def start_sim(self):
        self.start_btn.setEnabled(False)
        self.random_btn.setEnabled(False)
        self.start_kfs_state = copy.deepcopy(self.kfs_state) # Freeze starting state
        self.running = True
        self.status_lbl.setText(f"Status: Running (KFS: {self.kfs_count})")
        self.log("Simulation Started.")
        self.timer.start(1000)
        
    def reset_sim(self):
        self.timer.stop()
        self.running = False
        self.robot_pos = (4, 1)
        self.path_history = [(4, 1)]
        self.kfs_count = 0
        self.log_msgs.clear()
        self.known_maze = {i: "unknown" for i in range(1, 13)}
        self.start_btn.setEnabled(True)
        self.random_btn.setEnabled(True)
        self.kfs_state = copy.deepcopy(self.start_kfs_state) # Restore to original config
        for block_id, val in self.kfs_state.items():
            self.combos[block_id].setCurrentText(val)
        self.status_lbl.setText("Status: Setup (KFS: 0)")
        self.log_lbl.setText("Logs:\n")
        self.grid_widget_left.update()
        self.grid_widget_right.update()
        
    def check_block(self, pos):
        if pos == (-1, 0) or pos == (-1, 1) or pos == (-1, 2):
            return "exit"
        if pos not in self.COORDS_TO_BLOCK: 
            return "boundary"
        return self.kfs_state[self.COORDS_TO_BLOCK[pos]]
        
    def check_known_block(self, pos):
        if pos == (-1, 0) or pos == (-1, 1) or pos == (-1, 2):
            return "exit"
        if pos not in self.COORDS_TO_BLOCK: 
            return "boundary"
        state = self.known_maze[self.COORDS_TO_BLOCK[pos]]
        return "none" if state == "unknown" else state
        
    def get_best_move(self):
        import heapq
        
        # Deduce unknown blocks based on total R2 invariants (1 fake, 4 real)
        known_fake = sum(1 for state in self.known_maze.values() if state == "r2_fake")
        known_real = sum(1 for state in self.known_maze.values() if state == "r2_real") + self.kfs_count
        unknown_blocks = [b_id for b_id, state in self.known_maze.items() if state == "unknown"]
        
        inferred = {}
        if known_fake < 1 and len(unknown_blocks) == 1:
            inferred[unknown_blocks[0]] = "r2_fake"
        if known_real < 4 and len(unknown_blocks) == 4 - known_real:
            for b in unknown_blocks:
                inferred[b] = "r2_real"
                
        def get_inferred_state(pos):
            if pos == (-1, 0) or pos == (-1, 1) or pos == (-1, 2):
                return "exit"
            if pos not in self.COORDS_TO_BLOCK: 
                return "boundary"
            b_id = self.COORDS_TO_BLOCK[pos]
            if b_id in inferred:
                return inferred[b_id]
            state = self.known_maze[b_id]
            return "none" if state == "unknown" else state

        objective = "exit" if self.kfs_count == 2 else "harvest"

        # Dijkstra's Algorithm to find shortest weighted path
        # queue: (cost, step_counter, (r, c), path)
        queue = [(0, 0, self.robot_pos, [])]
        min_costs = {self.robot_pos: 0}
        step_counter = 0
        
        while queue:
            cost, _, (r, c), path = heapq.heappop(queue)
            
            if objective == "exit":
                if r == -1:
                    return path[0] if path else None
            else:
                # Harvest objective: Stop if we reached an unknown block or are adjacent to an r2_real
                if r != -1 and (r, c) in self.COORDS_TO_BLOCK:
                    b_id = self.COORDS_TO_BLOCK[(r, c)]
                    if self.known_maze[b_id] == "unknown" and inferred.get(b_id) != "r2_fake":
                        if path: return path[0]
                    
                    # Check if any adjacent is a known/inferred r2_real
                    adjacents = [(r+1, c), (r, c-1), (r, c+1), (r-1, c)]
                    for adj in adjacents:
                        if adj in self.COORDS_TO_BLOCK:
                            adj_b_id = self.COORDS_TO_BLOCK[adj]
                            if self.known_maze[adj_b_id] == "r2_real" or inferred.get(adj_b_id) == "r2_real":
                                if path: return path[0]
                
            neighbors = [(r-1, c), (r, c-1), (r, c+1), (r+1, c)]
            for nr, nc in neighbors:
                if nr > 4 or nc < 0 or nc > 2: continue # out of bounds
                
                target = (nr, nc)
                kfs = get_inferred_state(target)
                
                passable = False
                move_cost = 1.0
                
                if kfs == "exit":
                    if objective == "harvest":
                        continue # Do not exit if we still need to harvest!
                    passable = True
                elif kfs == "none":
                    passable = True
                elif kfs == "r1":
                    passable = True
                    move_cost = 2.5 # Base 1.0 + 1.5 wait penalty
                elif kfs == "r2_real" and self.kfs_count == 2:
                    # Can we relocate it? Need an adjacent empty space
                    adjacents = [(nr+1, nc), (nr, nc-1), (nr, nc+1), (nr-1, nc)]
                    for adj in adjacents:
                        if adj != (r, c) and get_inferred_state(adj) in ["none", "boundary"]:
                            passable = True
                            move_cost = 3.0 # Base 1.0 + 2.0 relocate penalty (worse than wait)
                            break
                            
                if passable:
                    # Sideways movement (detours) is extremely expensive
                    if nc != c:
                        move_cost += 10.0 # Moving > Relocating > Waiting
                        # Apply team bias as a tie-breaker when a detour is forced
                        if self.team == "Red":
                            if nc > c: move_cost -= 0.1
                            elif nc < c: move_cost += 0.1
                        else: # Blue
                            if nc < c: move_cost -= 0.1
                            elif nc > c: move_cost += 0.1
                            
                    new_cost = cost + move_cost
                    if target not in min_costs or new_cost < min_costs[target]:
                        min_costs[target] = new_cost
                        step_counter += 1
                        heapq.heappush(queue, (new_cost, step_counter, target, path + [target]))
                    
        return None
        
    def set_block(self, pos, val):
        if pos in self.COORDS_TO_BLOCK:
            block_id = self.COORDS_TO_BLOCK[pos]
            self.kfs_state[block_id] = val
        
    def pick_kfs(self, pos):
        if pos in self.COORDS_TO_BLOCK:
            block_id = self.COORDS_TO_BLOCK[pos]
            self.kfs_state[block_id] = "none"
            self.kfs_count += 1
            self.log(f"Picked r2_real from Block {block_id}. Count: {self.kfs_count}")
        
    def try_move(self, target):
        kfs = self.check_block(target)
        if kfs == "exit" or kfs == "none":
            self.move_to(target)
            return True
        elif kfs == "r2_real":
            if self.kfs_count < 2:
                # Fallback just in case, but harvest phase should handle this
                self.pick_kfs(target)
                self.move_to(target)
                return True
            else:
                # Count is 2. Pick and place somewhere else (relocate) without storing
                r, c = self.robot_pos
                adjacents = [(r+1, c), (r, c-1), (r, c+1), (r-1, c)]
                for adj in adjacents:
                    if adj != target and self.check_block(adj) in ["none", "boundary"]:
                        self.log(f"Relocated blocking r2_real to {adj}.")
                        self.set_block(target, "none")
                        if self.check_block(adj) == "none":
                            self.set_block(adj, "r2_real")
                            if adj in self.COORDS_TO_BLOCK:
                                self.known_maze[self.COORDS_TO_BLOCK[adj]] = "r2_real"
                        self.known_maze[self.COORDS_TO_BLOCK[target]] = "none"
                        self.move_to(target)
                        return True
                self.log("Cannot relocate r2_real, no empty adjacent blocks!")
                return False
        return False
        
    def move_to(self, pos):
        self.robot_pos = pos
        if pos not in self.path_history:
            self.path_history.append(pos)
            
        # Reveal the block we just stepped on
        if pos in self.COORDS_TO_BLOCK:
            b_id = self.COORDS_TO_BLOCK[pos]
            self.known_maze[b_id] = self.kfs_state[b_id]
            self.log(f"Moved to Block {b_id}")
            self.log(f"Moved to Block {COORDS_TO_BLOCK[pos]}")
        else:
            self.log(f"Moved to {pos}")
        self.grid_widget_left.update()
        self.grid_widget_right.update()
        
    def sim_step(self):
        if not self.running: return
        self.status_lbl.setText(f"Status: Running (KFS: {self.kfs_count})")
        
        r, c = self.robot_pos
        
        if r == 4:
            target = (3, 1) # Block 2
            # Entrance rule: we must move onto Block 2 to start.
            if not self.try_move(target):
                self.log(f"Waiting to enter Block 2 (blocked by {self.check_block(target)}).")
            return
            
        # Target block is (r-1, c) (Front is UP)
        front = (r-1, c)
        left = (r, c-1)
        right = (r, c+1)
        
        # 0. SENSOR SCAN PHASE (Update memory with adjacent blocks)
        for nr, nc in [front, left, right]:
            if (nr, nc) in self.COORDS_TO_BLOCK:
                block_id = self.COORDS_TO_BLOCK[(nr, nc)]
                self.known_maze[block_id] = self.kfs_state[block_id]
        self.grid_widget_right.update()
        
        # 1. HARVEST PHASE (Pick adjacent KFS without moving base)
        if self.kfs_count < 2:
            self.log(f"Scanning adjacent blocks from Block {self.COORDS_TO_BLOCK.get((r,c), (r,c))}...")
            
            # Scan order depends on team symmetry
            scan_order = [("Front", front), ("Right", right), ("Left", left)] if self.team == "Red" else [("Front", front), ("Left", left), ("Right", right)]
            
            for d, target in scan_order:
                if self.check_block(target) == "r2_real":
                    self.log(f"Found r2_real on {d}. Picking it up!")
                    self.pick_kfs(target)
                    return # End step, we harvested this tick. Wait for next tick to decide next move.
        
        # 2. MOVEMENT PHASE (Seek Exit or Harvest)
        if self.kfs_count == 2 and self.check_block(front) == "exit":
            self.log("Exited the forest!")
            self.status_lbl.setText(f"Status: Exited! (KFS: {self.kfs_count})")
            self.running = False
            self.move_to(front)
            return
            
        best_target = self.get_best_move()
        if best_target:
            self.try_move(best_target)
        else:
            if self.kfs_count < 2:
                self.log("Stuck! No reachable unknown blocks to explore. Waiting...")
            else:
                self.log("Stuck! No path to exit. Waiting for r1 removal...")

    def paint_grid(self, widget, event, state_dict, is_live=False):
        painter = QPainter(widget)
        w = widget.width() / 3
        h = widget.height() / 4
        
        # Draw blocks
        for block_id, (r, c) in self.BLOCKS.items():
            if is_live and self.known_maze[block_id] == "unknown":
                color = QColor(80, 80, 80, 200) # Dark Gray for Fog of War
            else:
                state = state_dict[block_id]
                if state == "r2_real": color = QColor(0, 255, 0, 100)
                elif state == "r2_fake": color = QColor(255, 0, 0, 100)
                elif state == "r1": color = QColor(0, 0, 255, 100)
                else: color = QColor(200, 200, 200, 50)
            
            painter.setPen(QPen(Qt.black, 1))
            painter.setBrush(QBrush(color))
            painter.drawRect(int(c*w), int(r*h), int(w), int(h))
            
            if is_live:
                painter.setPen(QPen(Qt.white if self.known_maze[block_id] == "unknown" else Qt.black))
                state_str = "Unknown" if self.known_maze[block_id] == "unknown" else state_dict[block_id]
                painter.drawText(int(c*w), int(r*h), int(w), int(h), Qt.AlignCenter, f"Block {block_id}\n{state_str}")
            
        # Draw path trace
        if len(self.path_history) > 1:
            painter.setPen(QPen(QColor("magenta"), 4, Qt.DashLine))
            for i in range(len(self.path_history) - 1):
                r1, c1 = self.path_history[i]
                r2, c2 = self.path_history[i+1]
                
                y1 = r1 * h + h/2 if 0 <= r1 < 4 else (4 * h if r1 == 4 else 0)
                x1 = c1 * w + w/2
                y2 = r2 * h + h/2 if 0 <= r2 < 4 else (4 * h if r2 == 4 else 0)
                x2 = c2 * w + w/2
                
                painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        # Draw robot
        rr, rc = self.robot_pos
        if rr == 4:
            y = 4 * h
            x = rc * w + w/2
        elif rr == -1:
            y = 0
            x = rc * w + w/2
        else:
            y = rr * h + h/2
            x = rc * w + w/2
            
        painter.setBrush(QBrush(QColor("purple")))
        painter.setPen(QPen(Qt.white, 2))
        painter.drawEllipse(int(x - 20), int(y - 20), 40, 40)
        painter.drawText(int(x - 10), int(y + 5), "R2")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ForestSimApp()
    ex.show()
    sys.exit(app.exec_())
