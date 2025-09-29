#!/usr/bin/env python3
import rclpy
import time
import threading
from statemachine import State, StateMachine
from pick_and_place.controller import Controller

class PickAndPlaceStateMachine(StateMachine):
    home = State("Home", initial=True)
    selecting_object = State("SelectingObject")                       
    picking_and_placing = State("PickingAndPlacing")                         
    done = State("Done", final=True)

    select_object = home.to(selecting_object, cond="are_objects_detected") | home.to(done, unless="are_objects_detected")
    pick_object = selecting_object.to(picking_and_placing, cond="object_selected")
    get_ready = picking_and_placing.to(home, cond="object_placed")
    
    def __init__(self, controller_node):
        self.controller = controller_node
        self.object_selected = False
        self.object_placed = False
        self.currently_selected_object = None

        self.logger = self.controller.get_logger()
        self.logger.info('\n' + 80*'=')
        self.logger.info("Pick-and-Place Mission Begins!") 
        self.logger.info(80*'=')

        super().__init__()

    def are_objects_detected(self) -> bool:
        time.sleep(1.0) 
        return self.controller.are_objects_on_workbench()

    def on_enter_home(self) -> None:
        self.logger.info("Moving to Home Position..") 
        self.controller.panda.move_to_neutral()
        self.object_selected = False
        self.object_placed = False
        time.sleep(0.2)
        self.send("select_object")

    def on_enter_selecting_object(self) -> None:
        self.logger.info("Selecting Object to Pick") 
        self.currently_selected_object = self.controller.select_random_object()
        self.object_selected = True
        self.logger.info("Object Selected.") 
        self.send("pick_object")
    
    def on_enter_picking_and_placing(self) -> None:
        self.logger.info("Starting Pick and Place Operation of Selected Object") 
        self.controller.move_object(self.currently_selected_object)
        self.object_placed = True
        self.logger.info("Object is Placed in its Bin.")
        self.send("get_ready")
    
    def on_enter_done(self) -> None:
        self.logger.info('\n' + 60*'=')
        self.logger.info("Mission Complete! \nAll Objects have been Placed in their Bins.") 
        self.logger.info(60*'=')

def main(args=None):
    rclpy.init(args=args)
    
    controller_node = Controller()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    state_machine = PickAndPlaceStateMachine(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()
    executor_thread.join()

if __name__ == "__main__":
    main()
