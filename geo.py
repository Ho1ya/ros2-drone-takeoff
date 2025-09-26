import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from my_msgs.action import PlanPath

def send_plan_path(start_xyz, goal_xyz, v_max=2.0, a_max=1.0, dt=0.1):
    rclpy.init()
    node = rclpy.create_node('geo_planpath_client')
    client = ActionClient(node, PlanPath, 'plan_path')

    goal_msg = PlanPath.Goal()
    goal_msg.start = Point(x=start_xyz[0], y=start_xyz[1], z=start_xyz[2])
    goal_msg.goal = Point(x=goal_xyz[0], y=goal_xyz[1], z=goal_xyz[2])
    goal_msg.v_max = v_max
    goal_msg.a_max = a_max
    goal_msg.dt = dt

    client.wait_for_server()
    future = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)
    node.destroy_node()
    rclpy.shutdown() 
    #вызов send_plan_path([0, 0, 0], [10, 5, 2])