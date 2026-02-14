#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode as Node
from rclpy.lifecycle import TransitionCallbackReturn,LifecycleState
from my_robot_interfaces.action import MoveTurtle
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill
from turtlesim.srv import Spawn
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading


class MoveTurtleNode(Node):
    def __init__(self):
        super().__init__("move_turtle")

        self.declare_parameter("name", 'turtle2')
        self.server_activate_ = False
        
        self.goal_lock_ =   threading.Lock()
        self.goal_done_event = threading.Event()

        self.goal_handle = ServerGoalHandle = None

        self.get_logger().info("the move_turtle node has been started")


 #-------------------LIFECYCLE_CALLBACKS------------------------------------------
    def on_configure(self, state:LifecycleState):   
        self.name = self.get_parameter("name").value
        self.client = self.create_client(Kill, '/kill') 
        self.client_spawn = self.create_client(Spawn, "/spawn")
        self.move_publisher = self.create_publisher(
            Twist, "/" + self.name + "/cmd_vel", 10)

        self.move_turtle_action = ActionServer(
            self,
            MoveTurtle,
            "move_turtle_" + self.name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())
        
        self.Kill_turtle()
        self.Spawn_turtle()
        
        self.get_logger().info("IN on_configure")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state:LifecycleState):    
        self.get_logger().info("Deactivated")
        self.client.destroy()
        self.client_spawn.destroy()
        self.move_publisher.destroy()
        self.move_turtle_action.destroy()
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state:LifecycleState):    
        self.get_logger().info("Activated")
        self.server_activate_ = True

        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state:LifecycleState):    
        self.get_logger().info("Deactivated")
        self.server_activate_ = False
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state:LifecycleState):    
        self.get_logger().info("Shutdown")
        self.server_activate_ = False
        self.Kill_turtle(self.name)
        self.client.destroy()
        self.client_spawn.destroy()
        self.move_publisher.destroy()
        self.move_turtle_action.destroy()
        return TransitionCallbackReturn.SUCCESS
        
    
 #-------------------TURTLE_CALLBACKS------------------------------------------
     
    def start_moving(self, linear_x, angular_z):

        if not self.server_activate_:
            self.get_logger().warn("node is not active")
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.move_publisher.publish(msg)

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.move_publisher.publish(msg)


    def stop_turtle(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.move_publisher.publish(msg)

    def Kill_turtle(self,turtle_name = 'turtle1'):
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /kill service...')

        request = Kill.Request()
        request.name = turtle_name

        future = self.client.call_async(request)
        future.add_done_callback(self.Kill_turtle_callback)

    def Kill_turtle_callback(self, future):
        response:Kill.Response = future.result()
        self.get_logger().info('Turtle killed')

    def Spawn_turtle(self):
        
        while not self.client_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')

        request = Spawn.Request()
        request.x = 5.0
        request.y = 5.0
        request.theta = 0.0
        request.name = self.name

        future = self.client_spawn.call_async(request)
        future.add_done_callback(self.spawn_turtle_callback)

    def spawn_turtle_callback(self, future):
        future.result()
        self.get_logger().info(self.name + " spawned")

    # ---------------- ACTION CALLBACKS ----------------

   
    def goal_callback(self, goal_request:MoveTurtle.Goal):
        self.get_logger().info('Goal request received')

        if not self.server_activate_:
            self.get_logger().warn("node not activated yet")
            return GoalResponse.REJECT

        #policy 1: If the current goal is active , Reject the new one
        with self.goal_lock_:
            if  self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().warn("the goal current goal is running so new goal has been rejected")
                return GoalResponse.REJECT

        if (abs(goal_request.linear_vel_x) > 3.0 or abs(goal_request.angular_vel_z) > 2.0 or goal_request.duration_sec <= 0) : 
            self.get_logger().error("the goal is being rejected")
            return  GoalResponse.REJECT
        
        else:
            self.get_logger().info("the goal is accepted")
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle:ServerGoalHandle):
        self.get_logger().info('Cancel request received')
        self.stop_turtle()
        self.move_timer.cancel()
        self.stop_timer.cancel()
        return CancelResponse.ACCEPT
  

    def execute_callback(self, goal_handle: ServerGoalHandle):

        self.goal_done_event.clear()


        linear_x = goal_handle.request.linear_vel_x
        angular_z = goal_handle.request.angular_vel_z
        period = goal_handle.request.duration_sec
        
        with self.goal_lock_:
            self.goal_handle = goal_handle


        self.move_timer = self.create_timer(
            0.1,
            lambda: self.start_moving(linear_x, angular_z)
        )

        self.stop_timer = self.create_timer(
            period,
            lambda: self.finish_goal(goal_handle)
        )
        

        self.get_logger().info('Executing turtle movement')
        
        self.goal_done_event.wait()
        
        result = MoveTurtle.Result()
        result.success = True
        result.message = "the turtle reached position"
        return result


    def finish_goal(self, goal_handle):
        self.stop_turtle()
        self.move_timer.cancel()
        self.stop_timer.cancel()

        goal_handle.succeed()

        with self.goal_lock_:
          self.goal_handle = None

        self.goal_done_event.set()

        self.get_logger().info('Goal succeeded')
        
   


def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtleNode()
    rclpy.spin(node,MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
