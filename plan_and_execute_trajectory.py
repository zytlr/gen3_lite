import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, RobotTrajectory
from moveit_msgs.action import ExecuteTrajectory
from shape_msgs.msg import SolidPrimitive

class PlanAndExecuteTrajectory(Node):
    def __init__(self):
        super().__init__('plan_and_execute_trajectory')

        # 경로 계획 서비스 클라이언트
        self.plan_client = self.create_client(GetMotionPlan, 'plan_kinematic_path')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Plan service not available, waiting...')

        # 경로 실행 액션 클라이언트
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.execute_client.wait_for_server()

    def plan_and_execute(self):
        # 경로 계획 요청
        plan_request = GetMotionPlan.Request()
        plan_request.motion_plan_request.group_name = "gen3_lite_arm"

        # 목표 위치 설정
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        # 값 바꿔보기
        pose.pose.position.x = 0.24411672353744507 #0.15599748492240906 #0.24411672353744507 #0.1
        pose.pose.position.y = 0.1436396837234497 #-0.43275338411331177 #0.1436396837234497 #-0.010
        pose.pose.position.z = 0.8186551332473755 #0.5680761337280273 #0.8186551332473755 #0.85
        # 얘네 4개는 고정
        pose.pose.orientation.x = 0.31965920329093933
        pose.pose.orientation.y = 0.6312269568443298
        pose.pose.orientation.z = 0.3188107907772064
        pose.pose.orientation.w = 0.6306585073471069

        # 경로 제약 조건 설정 (제약을 완화)
        position_constraint = PositionConstraint()
        position_constraint.header = pose.header
        position_constraint.link_name = "right_finger_prox_link"  # 목표 링크를 IK 가능 링크로 변경
        position_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.1, 0.1, 0.1]))  # 제약을 완화
        position_constraint.constraint_region.primitive_poses.append(pose.pose)

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        plan_request.motion_plan_request.goal_constraints.append(constraints)

        plan_future = self.plan_client.call_async(plan_request)
        rclpy.spin_until_future_complete(self, plan_future)

        # 경로 계획 결과 확인
        if plan_future.result() and plan_future.result().motion_plan_response.trajectory.joint_trajectory.points:
            self.get_logger().info('Motion plan found, executing trajectory...')
            planned_trajectory = plan_future.result().motion_plan_response.trajectory

            # 경로 실행 요청
            execute_goal = ExecuteTrajectory.Goal()
            execute_goal.trajectory = planned_trajectory

            execute_future = self.execute_client.send_goal_async(execute_goal)
            rclpy.spin_until_future_complete(self, execute_future)

            result_future = execute_future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            if result_future.result():
                self.get_logger().info('Trajectory execution complete')
            else:
                self.get_logger().info('Trajectory execution failed')
        else:
            self.get_logger().info('Failed to find motion plan')

def main(args=None):
    rclpy.init(args=args)
    node = PlanAndExecuteTrajectory()
    node.plan_and_execute()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

