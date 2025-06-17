import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

from pharmacy_msgs.srv import GetMedicineName, PickupMedicine, SrvDepthPosition

AVAILABLE_DRUGS = [
    "모드콜", "콜대원", "하이펜", "타이레놀", "다제스",
    "락토프린", "포비돈", "미니온", "퓨어밴드", "Rohto C3 Cube"
]

class PharmacyManager(Node):
    def __init__(self):
        super().__init__('pharmacy_manager')

        # 사용자 발화 구독
        self.subscription = self.create_subscription(
            String,
            '/symptom_text',
            self.symptom_callback,
            10
        )

        # 서비스 클라이언트 설정
        self.get_medicine_client = self.create_client(GetMedicineName, '/get_medicine_name')
        self.detect_position_client = self.create_client(SrvDepthPosition, '/get_3d_position')
        self.pickup_client = self.create_client(PickupMedicine, '/pickup_medicine')

        # 추천 결과 퍼블리시 (GUI용)
        self.recommend_pub = self.create_publisher(String, '/recommended_drug', 10)

        self.get_logger().info("PharmacyManager 실행됨 — 사용자 입력 대기 중")

    def symptom_callback(self, msg: String):
        user_input = msg.data.strip()

        # 1. 약 이름 직접 언급 시 → 즉시 처리
        # if user_input in AVAILABLE_DRUGS:
        #     self.get_logger().info(f"약 이름 직접 언급됨: {user_input}")
        #     self.process_medicine(user_input)
        #     return

        # 2. 증상 입력 종료 신호
        if user_input == "__DONE__":
            try:
                path = os.path.expanduser("~/ros2_ws/src/pharmacy_main/resource/symptom_query.txt")
                with open(path, "r", encoding="utf-8") as f:
                    symptom_text = f.read().strip()
                self.get_logger().info(f"최종 증상 텍스트: {symptom_text}")
            except Exception as e:
                self.get_logger().error(f"symptom_query.txt 로딩 실패: {e}")
                return

            # 추천 약 요청 → 리스트로 받음
            recommended = self.call_get_medicine_name(symptom_text)
            if not recommended:
                self.get_logger().info("약 추천 대기중...")
                return

            for drug in recommended:
                self.get_logger().info(f"추천된 약: {drug}")
                self.process_medicine(drug)

                # 결과를 GUI에도 보여주기 위해 전체 추천 결과 퍼블리시 (최초 한 번만)
            self.recommend_pub.publish(String(data=", ".join(recommended)))
            return

        # 그 외 일반 텍스트 (참고용 로그)
        self.get_logger().info(f"(참고용) 입력 수신: \"{user_input}\"")

    def process_medicine(self, medicine_name: str):
        result = self.call_detect_position(medicine_name)
        if not result:
            self.get_logger().error("약 위치 탐지 실패")
            return

        position, width = result
        point = Point()
        point.x, point.y, point.z = position
        point.orientation.w = 1.0  # 기본값 설정

        self.get_logger().info(f"{medicine_name}의 위치: {point}")
        self.get_logger().info(f"{medicine_name}의 폭: {width[0]}")
        self.get_logger().info(f"{medicine_name}: {type(width)}")

        success = self.call_pickup([point], width)
        if success:
            self.get_logger().info(f"{medicine_name} 집기 성공")
        else:
            self.get_logger().error(f"{medicine_name} 집기 실패")

    def call_get_medicine_name(self, symptom: str) -> list:
        if not self.get_medicine_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/get_medicine_name 서비스 연결 실패")
            return []

        request = GetMedicineName.Request()
        request.symptom = symptom
        future = self.get_medicine_client.call_async(request)

        # 비동기 처리로 변경
        def response_callback(fut):
            try:
                result = fut.result()
                if result and result.medicine:
                    self.get_logger().info(f"추천된 약: {result.medicine}")
                    self.process_medicine(result.medicine)

                else:
                    self.get_logger().info("약 정보 대기중")
            except Exception as e:
                self.get_logger().error(f"서비스 응답 처리 중 예외 발생: {e}")
        future.add_done_callback(response_callback)


        # rclpy.spin_until_future_complete(self, future)
        return future.result().medicine if future.result() else None


    def call_detect_position(self, medicine: str):
        if not self.detect_position_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/get_3d_position 서비스 연결 실패")
            return None

        request = SrvDepthPosition.Request()
        request.target = medicine
        future = self.detect_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if not result or sum(result.depth_position) == 0.0:
            return None

        return result.depth_position, result.width

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

