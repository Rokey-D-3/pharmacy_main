import os
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

from pharmacy_msgs.srv import GetMedicineName, PickupMedicine, SrvDepthPosition

AVAILABLE_DRUGS = [
    "모드콜", "콜대원", "하이펜", "타이레놀", "다제스",
    "락토프린", "포비돈", "미니온", "퓨어밴드", "rohto c3 cube"
]

class PharmacyManager(Node):
    def __init__(self):
        super().__init__('pharmacy_manager')

        # 증상 또는 약 이름 수신
        self.subscription = self.create_subscription(
            String,
            '/symptom_text',
            self.symptom_callback,
            10
        )

        # 필요한 서비스 클라이언트 생성
        self.get_medicine_client = self.create_client(GetMedicineName, '/get_medicine_name')
        self.detect_position_client = self.create_client(SrvDepthPosition, '/get_3d_position')
        self.pickup_client = self.create_client(PickupMedicine, '/pickup_medicine')
        self.return_home_client = self.create_client(Trigger, '/return_home')

        self.recommend_pub = self.create_publisher(String, '/recommended_drug', 10)
        
        self.get_logger().info("PharmacyManager 실행됨 — 사용자 입력 대기 중")

    def symptom_callback(self, msg: String):
        user_input = msg.data.strip()

        # 1. 약 이름이면 바로 집기  <- 삭제 예정
        # if user_input in AVAILABLE_DRUGS:
        #     self.get_logger().info(f"약 이름 직접 언급됨: {user_input}")
        #     user_input = [user_input]
        #     self.process_medicine(user_input)
        #     return

        # 2. '__DONE__' 신호가 오면 symptom_query.txt 읽고 약 추천 수행
        if user_input == "__DONE__":
            try:
                path = os.path.expanduser("~/ros2_ws/src/pharmacy_main/resource/symptom_query.txt")
                with open(path, "r", encoding="utf-8") as f:
                    symptom_text = f.read().strip()
                self.get_logger().info(f"최종 증상 텍스트: {symptom_text}")
            except Exception as e:
                self.get_logger().error(f"symptom_query.txt 로딩 실패: {e}")
                return

            recommended = self.call_get_medicine_name(symptom_text)

            if not recommended:
                self.get_logger().info("약 추천 대기중...")
                return
            
            # self.get_logger().info(f"추천된 약: {recommended}")



            # gui_msg = ", ".join(recommended)
            # self.recommend_pub.publish(String(data=gui_msg))
            # print('debugging11')
            # for num, recommend in enumerate(recommended):
            # #여기서 for문으로 개별 처리
            #     self.get_logger().info(f"{num+1}번째 : {recommend} 프로세스 실행....") 
            #     self.process_medicine([recommend])
            # ##

            # print('debugging22')
            return

        # 그 외에는 단순 텍스트 로그만
        self.get_logger().info(f"(참고용) 입력 수신: \"{user_input}\"")

    def return_robot_home(self):
        if not self.return_home_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/return_home 서비스 연결 실패")
            return
        req = Trigger.Request()
        future = self.return_home_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info("로봇 홈 복귀 완료")
        else:
            self.get_logger().error("로봇 홈 복귀 실패")
    
    def process_medicine(self, medicine_name: str, retry: int = 2):
        for attempt in range(retry + 1):
            try:
                self.get_logger().info(f"[{medicine_name}] 처리 시도 {attempt+1}/{retry+1}")

                result = self.call_detect_position(medicine_name)
                if not result or self._is_invalid_position(result[0]):
                    self.get_logger().warn(f"[{medicine_name}] 감지 실패")
                    self.return_robot_home()
                    if attempt < retry:
                        time.sleep(1.5)
                    continue

                position, width = result
                self.get_logger().info(f"{medicine_name}의 위치: {position}")
                self.get_logger().info(f"{medicine_name}의 폭: {width}")

                success = self.call_pickup(position, width)
                if success:
                    self.get_logger().info("약 집기 성공")
                    return
                else:
                    self.get_logger().warn(f"{medicine_name} 집기 실패 → 감지부터 재시도 예정")
                    self.return_robot_home()
                    if attempt < retry:
                        time.sleep(1.5)
                    continue

            except Exception as e:
                self.get_logger().error(f"{medicine_name} 처리 중 예외 발생: {e}")
                self.return_robot_home()
                if attempt < retry:
                    time.sleep(1.5)
                continue

        self.get_logger().error(f"{medicine_name} 처리 실패 — 최종 포기")


    def _is_invalid_position(self, points: list[Point]) -> bool:
        p = points[0]
        return p.x == 0.0 and p.y == 0.0 and p.z == 0.0
    
    def call_get_medicine_name(self, symptom: str) -> str:
        if not self.get_medicine_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/get_medicine_name 서비스 연결 실패")
            return None

        request = GetMedicineName.Request()
        request.symptom = symptom
        future = self.get_medicine_client.call_async(request)


        # 비동기 처리로 변경
        def response_callback(fut):
            try:
                result = fut.result()
                if result and result.medicine:
                    recommended = result.medicine

                    self.get_logger().info(f"-추천된 약 : {recommended}")

                    gui_msg = ", ".join(recommended)
                    self.recommend_pub.publish(String(data=gui_msg))
                    print('debugging11')
                    for num, recommend in enumerate(recommended):
                    #여기서 for문으로 개별 처리
                        self.get_logger().info(f"{num+1}번째 : {recommend} 프로세스 실행....") 
                        self.process_medicine([recommend])

                    # 종료 확인용 dummy data
                    dummy_point = [Point()]
                    dummy_width = [10001]
                    self.get_logger().info("모든 약 처리 후 serve() 트리거 전송")
                    self.call_pickup(dummy_point, dummy_width)
                    # self.process_medicine(result.medicine)

                else:
                    self.get_logger().info("약 정보 대기중")
            except Exception as e:
                self.get_logger().error(f"서비스 응답 처리 중 예외 발생: {e}")
        future.add_done_callback(response_callback)


        # rclpy.spin_until_future_complete(self, future)
        # return future.result().medicine if future.result() else None

    def call_detect_position(self, medicine):
        try:
            if not self.detect_position_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("/get_3d_position 서비스 연결 실패")
                return None

            request = SrvDepthPosition.Request()
            request.target = medicine
            future = self.detect_position_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            result = future.result()

            # 결과가 없거나 비어있으면 실패로 간주
            if result is None or not result.depth_position:
                self.get_logger().warn(f"{medicine} 위치 탐지 실패 — 응답 없음")
                return None

            return result.depth_position, result.width

        except Exception as e:
            self.get_logger().error(f"{medicine} 감지 중 예외 발생: {e}")
            return None

    def call_pickup(self, point: Point, width) -> bool:
        if not self.pickup_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/pickup_medicine 서비스 연결 실패")
            return False

        request = PickupMedicine.Request()
        request.point = point
        request.width = width
        future = self.pickup_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success if future.result() else False


def main(args=None):
    rclpy.init(args=args)
    node = PharmacyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

