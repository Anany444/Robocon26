#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import heapq

from robocon_interfaces.srv import GetHighLevelPlan
from std_msgs.msg import String

class HighLevelPlanner(Node):
    def __init__(self):
        super().__init__('high_level_planner')
        
        self.srv = self.create_service(GetHighLevelPlan, '/get_high_level_plan', self.plan_callback)
        self.mem_sub = self.create_subscription(String, '/planner/update_memory', self.memory_callback, 10)
        
        # Internal state memory
        self.known_maze = {i: "unknown" for i in range(1, 13)}
        
        # Coordinate mapping (Team Red by default)
        self.team = "Red"
        self.BLOCKS = {
            10: (0, 0), 11: (0, 1), 12: (0, 2),
            7:  (1, 0), 8:  (1, 1), 9:  (1, 2),
            4:  (2, 0), 5:  (2, 1), 6:  (2, 2),
            1:  (3, 0), 2:  (3, 1), 3:  (3, 2)
        }
        self.COORDS_TO_BLOCK = {v: k for k, v in self.BLOCKS.items()}
        
        self.get_logger().info("High-Level Planner (Dijkstra) initialized and ready.")

    def memory_callback(self, msg):
        try:
            b_id_str, k_type = msg.data.split(':')
            b_id = int(b_id_str)
            self.known_maze[b_id] = k_type
            self.get_logger().info(f"Memory Updated: Block {b_id} is now {k_type}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse memory update: {msg.data}")

    def get_best_move(self, robot_pos, kfs_count):
        # 1. Deduce unknowns (1 fake, 4 real invariant)
        known_fake = sum(1 for state in self.known_maze.values() if state == "r2_kfs_fake")
        known_real = sum(1 for state in self.known_maze.values() if state == "r2_kfs_real") + kfs_count
        unknown_blocks = [b_id for b_id, state in self.known_maze.items() if state == "unknown"]
        
        inferred = {}
        if known_fake < 1 and len(unknown_blocks) == 1:
            inferred[unknown_blocks[0]] = "r2_kfs_fake"
        if known_real < 4 and len(unknown_blocks) == 4 - known_real:
            for b in unknown_blocks:
                inferred[b] = "r2_kfs_real"
                
        def get_inferred_state(pos):
            if pos == (-1, 0) or pos == (-1, 2): return "exit"
            if pos == (-1, 1): return "boundary"
            if pos not in self.COORDS_TO_BLOCK: return "boundary"
            b_id = self.COORDS_TO_BLOCK[pos]
            if b_id in inferred: return inferred[b_id]
            state = self.known_maze[b_id]
            return "none" if state == "unknown" else state

        objective = "exit" if kfs_count == 2 else "harvest"

        queue = [(0, 0, robot_pos, [])]
        min_costs = {robot_pos: 0}
        step_counter = 0
        
        while queue:
            cost, _, (r, c), path = heapq.heappop(queue)
            
            if objective == "exit":
                if r == -1:
                    return path[0] if path else None
            else:
                if r != -1 and (r, c) in self.COORDS_TO_BLOCK:
                    b_id = self.COORDS_TO_BLOCK[(r, c)]
                    if self.known_maze[b_id] == "unknown" and inferred.get(b_id) != "r2_kfs_fake":
                        if path: return path[0]
                    
                    adjacents = [(r+1, c), (r, c-1), (r, c+1), (r-1, c)]
                    for adj in adjacents:
                        if adj in self.COORDS_TO_BLOCK:
                            adj_b_id = self.COORDS_TO_BLOCK[adj]
                            if self.known_maze[adj_b_id] == "r2_kfs_real" or inferred.get(adj_b_id) == "r2_kfs_real":
                                if path: return path[0]
                
            neighbors = [(r-1, c), (r, c-1), (r, c+1), (r+1, c)]
            for nr, nc in neighbors:
                if nr > 4 or nc < 0 or nc > 2: continue
                
                target = (nr, nc)
                kfs = get_inferred_state(target)
                
                passable = False
                move_cost = 1.0
                
                if kfs == "exit":
                    if objective == "harvest": continue
                    passable = True
                elif kfs == "none":
                    passable = True
                elif kfs == "r1":
                    passable = True
                    move_cost = 2.5
                elif kfs == "r2_real" and kfs_count == 2:
                    adjacents = [(nr+1, nc), (nr, nc-1), (nr, nc+1), (nr-1, nc)]
                    for adj in adjacents:
                        if adj != (r, c) and get_inferred_state(adj) in ["none", "boundary"]:
                            passable = True
                            move_cost = 3.0
                            break
                            
                if passable:
                    if nc != 1 and target != (-1, 1): 
                        move_cost += 5.0
                    if nc != c:
                        move_cost += 10.0
                        if self.team == "Red":
                            if nc > c: move_cost -= 0.1
                            elif nc < c: move_cost += 0.1
                        else:
                            if nc < c: move_cost -= 0.1
                            elif nc > c: move_cost += 0.1
                            
                    new_cost = cost + move_cost
                    if target not in min_costs or new_cost < min_costs[target]:
                        min_costs[target] = new_cost
                        step_counter += 1
                        heapq.heappush(queue, (new_cost, step_counter, target, path + [target]))
                    
        return None

    def get_relative_direction(self, current_pos, target_pos, current_facing_block_id):
        # absolute direction of target
        dr = target_pos[0] - current_pos[0]
        dc = target_pos[1] - current_pos[1]
        
        if dr == -1: abs_dir = "north" # UP
        elif dr == 1: abs_dir = "south" # DOWN
        elif dc == 1: abs_dir = "east" # RIGHT
        elif dc == -1: abs_dir = "west" # LEFT
        else: return "front"
        
        # calculate absolute direction we are currently facing
        facing_pos = self.BLOCKS.get(current_facing_block_id, current_pos)
        f_dr = facing_pos[0] - current_pos[0]
        f_dc = facing_pos[1] - current_pos[1]
        if f_dr == -1: current_facing = "north"
        elif f_dr == 1: current_facing = "south"
        elif f_dc == 1: current_facing = "east"
        elif f_dc == -1: current_facing = "west"
        else: current_facing = "north"
        
        dirs = ["north", "east", "south", "west"]
        idx_curr = dirs.index(current_facing)
        idx_targ = dirs.index(abs_dir)
        
        diff = (idx_targ - idx_curr) % 4
        if diff == 0: return "front"
        if diff == 1: return "right"
        if diff == 2: return "back"
        if diff == 3: return "left"

    def plan_callback(self, request, response):
        self.get_logger().info(f"Received BT Request: current_block_id={request.current_block_id}, current_facing_block_id={request.current_facing_block_id}, current_kfs_count={request.current_kfs_count}, gripper_has_kfs={request.gripper_has_kfs}")

        rpos = self.BLOCKS.get(request.current_block_id, (4, 1))
        r, c = rpos
        
        # 1. State machine priority: If holding a KFS, process it!
        if request.gripper_has_kfs:
            if getattr(self, 'is_relocating', False):
                # We need to place it on an empty block
                adjacents = [(r-1, c), (r, c-1), (r, c+1), (r+1, c)]
                for adj in adjacents:
                    if adj in self.COORDS_TO_BLOCK:
                        adj_id = self.COORDS_TO_BLOCK[adj]
                        if self.known_maze[adj_id] == "none":
                            req_dir = self.get_relative_direction(rpos, adj, request.current_facing_block_id)
                            if req_dir == "front":
                                response.sequence_name = "place_picked_kfs_on_faced_block"
                                response.sequence_params_json = json.dumps({"target_block_id": adj_id})
                                self.is_relocating = False
                            else:
                                response.sequence_name = "face_towards_an_adjacent_block"
                                response.sequence_params_json = json.dumps({"direction": req_dir, "target_block_id": adj_id})
                            return response
            else:
                # Normal harvest -> Store it
                response.sequence_name = "store_picked_kfs_on_bot"
                response.sequence_params_json = json.dumps({"storage_slot": request.current_kfs_count + 1})
                return response

        # 2. Is there an R2_REAL adjacent and we need to harvest?
        adjacents = [(r-1, c), (r, c-1), (r, c+1), (r+1, c)]
        if request.current_kfs_count < 2:
            for adj in adjacents:
                if adj in self.COORDS_TO_BLOCK:
                    adj_id = self.COORDS_TO_BLOCK[adj]
                    if self.known_maze[adj_id] == "r2_kfs_real":
                        req_dir = self.get_relative_direction(rpos, adj, request.current_facing_block_id)
                        if req_dir == "front":
                            response.sequence_name = "pick_kfs_from_faced_block"
                            response.sequence_params_json = json.dumps({"kfs_type": "r2_kfs_real"})
                            self.is_relocating = False
                        else:
                            response.sequence_name = "face_towards_an_adjacent_block"
                            response.sequence_params_json = json.dumps({"direction": req_dir, "target_block_id": adj_id})
                        return response

        # 3. Do we need to scan unknowns?
        if request.current_kfs_count < 2:
            for adj in adjacents:
                if adj in self.COORDS_TO_BLOCK:
                    adj_id = self.COORDS_TO_BLOCK[adj]
                    if self.known_maze[adj_id] == "unknown":
                        req_dir = self.get_relative_direction(rpos, adj, request.current_facing_block_id)
                        if req_dir == "front":
                            response.sequence_name = "scan_the_faced_block"
                            response.sequence_params_json = json.dumps({}) # No params needed for scan
                        else:
                            response.sequence_name = "face_towards_an_adjacent_block"
                            response.sequence_params_json = json.dumps({"direction": req_dir, "target_block_id": adj_id})
                        return response

        # 4. Ask Dijkstra for the next target
        target_pos = self.get_best_move(rpos, request.current_kfs_count)
        
        if not target_pos:
            response.sequence_name = "wait_for_r1_kfs_pick"
            response.sequence_params_json = json.dumps({})
            return response
            
        target_id = self.COORDS_TO_BLOCK.get(target_pos, 0)
        
        # 5. We have a target! Are we facing it?
        req_dir = self.get_relative_direction(rpos, target_pos, request.current_facing_block_id)
        if req_dir != "front":
            response.sequence_name = "face_towards_an_adjacent_block"
            response.sequence_params_json = json.dumps({"direction": req_dir, "target_block_id": target_id})
            return response
            
        # 6. We are facing the target. Is it blocked?
        kfs = "exit" if target_pos[0] == -1 else self.known_maze.get(target_id, "none")
        if kfs == "exit":
            response.sequence_name = "exit_forest"
            response.sequence_params_json = json.dumps({"current_block_id": request.current_block_id, "exit_ramp_id": 1})
            return response
            
        if kfs == "r1_kfs":
            response.sequence_name = "wait_for_r1_kfs_pick"
            response.sequence_params_json = json.dumps({"target_block_id": target_id, "timeout_seconds": 10.0, "retry_interval_seconds": 1.0})
            return response
            
        if kfs == "r2_kfs_real" and request.current_kfs_count == 2:
            # We need to relocate it!
            response.sequence_name = "pick_kfs_from_faced_block"
            response.sequence_params_json = json.dumps({"kfs_type": "r2_kfs_real"})
            self.is_relocating = True
            return response

        # 7. Default: Move!
        response.sequence_name = "move_from_current_block_to_faced_block"
        response.sequence_params_json = json.dumps({
            "current_block_id": request.current_block_id,
            "target_block_id": target_id
        })
        return response

def main(args=None):
    rclpy.init(args=args)
    node = HighLevelPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
