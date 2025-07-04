import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
# from std_msgs.msg import UInt16
from pharmacy_msgs.srv import PickupMedicine  # Point[], UInt16[] → bool 응답 서비스
from std_srvs.srv import Trigger

import time

# OnRobot RG2 그리퍼 제어용 클래스
from pharmacy_main.robot_control.onrobot import RG

# DSR 로봇 API
import DR_init

# ─────────────── 로봇 및 그리퍼 설정 ───────────────
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 80, 80

# OnRobot RG2 그리퍼 설정
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

WIDTH_MARGIN = 80
GRIP_MARGIN = 10
TARGET_MOVING_MARGIN = 50
STAB_DISTANCE = 40
DRAW_DISTANCE = 220
# DRAW_DISTANCE = 100

PUT_FORCE = 100
CHK_PUT_FORCE = 10
# ─────────────── DSR API 초기화 ───────────────
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init(args=None)  # ROS 2 초기화
dsr_node = rclpy.create_node("robot_arm_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import(
        posx,
        posj,
        movej, 
        movel, 
        get_current_posx, 
        mwait, 
        set_tool, 
        set_tcp,
        task_compliance_ctrl,
        set_desired_force,
        set_stiffnessx,
        release_force,
        release_compliance_ctrl,
        check_force_condition,
        DR_MV_MOD_REL, DR_FC_MOD_ABS, DR_AXIS_Z, DR_BASE, DR_TOOL,
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")

class RobotArm(Node):
    """
    /pickup_medicine 서비스를 통해 약 위치(Point)를 받아
    로봇팔이 이동 및 집기를 수행한 후 응답을 반환하는 ROS 2 노드
    """
   
    def __init__(self):
        super().__init__('robot_arm')

        # pickup_medicine 서비스 서버 생성
        self.srv = self.create_service(
            PickupMedicine,
            '/pickup_medicine',
            self.pickup_callback
        )
        self.create_service(Trigger, '/return_home', self.return_home_callback)  
        self.get_logger().info('Robot Arm 서비스 시작됨 (/pickup_medicine, /return_home)')

        # 로봇 초기 자세로 이동
        self.init_robot()

    def init_robot(self) -> None:
        """
        로봇팔을 초기 관절 자세로 이동시키고 그리퍼를 닫아서 이동 준비
        """
        set_tool("TW_brkt")       # 사전에 등록된 툴 이름
        set_tcp("RG2_brkt")       # 사전에 등록된 TCP 이름

        self.move_home()

        gripper.close_gripper(400)
        self.gripper_wait_busy()
        
        '''
        # # test code
        # target = Point()
        # target.x = 140.98
        # target.y = -898.71
        # target.z = 380.29
        # width = 710

        # self.pick_target(target, width)
        # self.put_target(width)

        # self.serve()
        '''
    
    def move_home(self) -> None:
        # Default Home position
        # movej([0,0,90,0,90,0], vel=VELOCITY, acc=ACC)
        # mwait()

        # Home position 1
        Home = posj([-139.43, -33.86, 146.12, 96.24, 48.81, -20.92])
        # Home position 2. posj([0,0,90,0,90,0]) 에서 너무 많이 움직임
        # Home = posj([45.94, 34.72, -146.17, -81.9, 43.76, -20.66])
        
        movej(Home, vel=VELOCITY, acc=ACC)
        mwait()
        
    def return_home_callback(self, request, response):
        self.get_logger().info("홈 위치로 복귀 요청 수신 → move_home 실행")
        self.move_home()
        response.success = True
        response.message = "홈 복귀 완료"
        return response
    
    def move_rel(self, x, y, z, vel=VELOCITY, acc=ACC) -> None:
        movel(pos=[x, y, z, 0, 0, 0], vel=vel, acc=acc, mod=DR_MV_MOD_REL)
        mwait()

    def gripper_wait_busy(self) -> None:
        while True:
            status = gripper.get_status()
            if status[0] is None:
                print("Status read error.")
                break
            if not status[0]:
                break
            time.sleep(.3)

    def gripper_wait_grip(self) -> None:
        while True:
            status = gripper.get_status()
            # if status[1] or status[0] is None:
            if status[1] is None:
                print("Status read error.")
                break
            # if status[1] and (not status[0]):
            if status[1]:
                break
            time.sleep(.3)

    def chk_fc_z(self, force=10) -> None:
        while \
            not check_force_condition(axis=DR_AXIS_Z, max=force, ref=DR_TOOL):
            pass

    def set_fc(self, axis, force) -> None:
        stfns_tra = 20_000.
        stfns_rot = 1_000.
        task_compliance_ctrl()
        time.sleep(0.1)
        if axis == 'x':
            set_stiffnessx(
                [0., stfns_tra, stfns_tra, stfns_rot, stfns_rot, stfns_rot], 
                time=0.0)
            set_desired_force([force, 0., 0., 0., 0., 0.], 
                              [1, 0, 0, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)
        elif axis == 'y':
            set_stiffnessx(
                [stfns_tra, 0., stfns_tra, stfns_rot, stfns_rot, stfns_rot], 
                time=0.0)
            set_desired_force([0., force, 0., 0., 0., 0.], 
                              [0, 1, 0, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)
        elif axis == 'z':
            set_stiffnessx(
                [stfns_tra, stfns_tra, 0., stfns_rot, stfns_rot, stfns_rot], 
                time=0.0)
            set_desired_force([0., 0., force, 0., 0., 0.], 
                              [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

    def release_fc(self) -> None:
        release_force()
        release_compliance_ctrl()

    def pick_target(self, target: Point, width: int) -> bool:
        # target; Point()
        # with; Int8()
        
        print(f"x:{target.x}, y:{target.y}, z:{target.z}")
        # moving HOME_R or HOME_L
        HOME_R = posj([-110.18, 23.90, 104.89, 149.61, 42.97, -66.80])
        # posx([-150, -780, 280, 90, -90, -90])
        HOME_L = posj([-78.95, 22.08, 107.49, 17.03, -40.87, 76.95])
        # posx([90, -780, 280, 90, -90, -90])
        if target.x > 0:
            movej(HOME_L, vel=VELOCITY, acc=ACC)
        else:
            movej(HOME_R, vel=VELOCITY, acc=ACC)
        mwait()

        # gripper move with target with + margin
        gripper.move_gripper(width + WIDTH_MARGIN, 400)
        self.gripper_wait_busy()
        
        # movel target XZ + Y+TARGET_MOVING_MARGIN
        # print current pose for check
        print(get_current_posx()[0])

        movel(posx([target.x, target.y+TARGET_MOVING_MARGIN, target.z, 90, -90, -90]), vel=VELOCITY, acc=ACC)
        mwait()

        # movel target Y
        self.move_rel(0, -TARGET_MOVING_MARGIN-STAB_DISTANCE, 0)
        # movel(posx([target.x, target.y, target.z, 90, -90, -90]), vel=VELOCITY, acc=ACC)
        mwait()

        # gripper grip with width
        # need to optimize gripping force
        gripper.close_gripper(100)
        self.gripper_wait_grip()
       
        # relative move Y+3,Z+10
        # lift motion
        self.move_rel(0, 3, 10)
        mwait()

        # movel gripped pill box DRAW motion
        movel(posx([target.x, target.y+DRAW_DISTANCE, target.z, 90, -90, -90]), vel=VELOCITY, acc=ACC)
        mwait()
        gripper_width = gripper.get_width()
        print(gripper_width)
        # determine if the pill box is grapped
        if gripper_width < 9.9:
            return False
        else:
            return True

    def put_target(self, width: int) -> None:
        # move to put place
        MIDIUM = posj([-62.69, -0.45, 117.50, 49.52, -37.47, 47.18])
        # PUT = posj([-0.71, 18.93, 74.09, -0.03, 86.98, 269.29])
        PUT = posj([-0.19, 22.57, 53.78, -0.04, 103.13, 269.71])
        # posx([500, 0, 150, 0, -180, -90])
        movej(MIDIUM, vel=VELOCITY, acc=ACC)
        mwait()
        movej(PUT, vel=VELOCITY, acc=ACC)
        mwait()
        # compliance/force ctrl on Z axis
        self.set_fc('z', -PUT_FORCE)
        # check force Z axis
        self.chk_fc_z(CHK_PUT_FORCE)
        # compliance/force ctrl off
        self.release_fc()
        gripper.move_gripper(width + WIDTH_MARGIN, 400)
        self.gripper_wait_busy()
        movej(PUT, vel=VELOCITY, acc=ACC)

    def serve(self) -> None:
        SERVE1 = posj([-0.99, 8.81, 117.03, -0.02, 54.16, 269.02])
        # posx([360, 0, -28.5, 90, -180, 0])
        # SERVE2 = posj([-0.55, 46.01, 58.91, -0.03, 75.07, 269.45])
        # posx([650, 0, -28.5, 0, -180, -90])
        PUT = posj([-0.71, 18.93, 74.09, -0.03, 86.98, 269.29])

        gripper.open_gripper(400)
        self.gripper_wait_busy()
        
        movej(SERVE1, vel=VELOCITY, acc=ACC)
        mwait()
        # movej(SERVE2, vel=VELOCITY, acc=ACC)
        self.move_rel(290, 0, 0, 100, 100)
        mwait()
        movej(PUT, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.close_gripper(400)
        self.gripper_wait_busy()


    def pickup_callback(self, request, response):
        """
        서비스 콜백:
        position을 기반으로 좌표 변환 및 보정
        약 위치로 로봇팔 이동
        그리퍼로 약 집기 -> 성공 여부 반환
        약을 집은 후 다시 초기 위치로 복귀
        """
        
        target = request.point[0]
        width = request.width[0]
        self.get_logger().info(f"targets{target}")
        self.get_logger().info(f"widths{width}")
        if width > 10_000:
            self.serve()
        else:
            try:
                assert ( target.x or target.y or target.z ), "target error"
                try:
                    assert self.pick_target(target, width), "failed to picking pill box"
                    self.put_target(width)
                    response.success = True
                    self.get_logger().info("완료")
                except AssertionError as e:
                    self.get_logger().error(f"실패: {str(e)}")
                    self.init_robot()
                    response.success = False
                except Exception as e:
                    self.get_logger().error(f"실패: {str(e)}")
                    self.init_robot()
                    response.success = False
            except AssertionError as e:
                self.get_logger().error(f"실패: {str(e)}")
                self.init_robot()
                response.success = False

        self.init_robot()
        return response

def main(args=None):
    node = RobotArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("로봇 제어 노드 종료됨")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()