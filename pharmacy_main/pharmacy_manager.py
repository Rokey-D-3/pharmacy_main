import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from rclpy.executors import MultiThreadedExecutor

from pharmacy_msgs.srv import GetMedicineName, PickupMedicine, SrvDepthPosition

AVAILABLE_DRUGS = [
    "모드콜", "콜대원", "하이펜", "타이레놀", "다제스",
    "락토프린", "포비돈", "미니온", "퓨어밴드", "rohto c3 cube"
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

        self.recommended_list = []  # 전체 약 리스트 저장
        self.current_index = 0      # 현재 처리 중인 약 인덱스

        self.get_logger().info("PharmacyManager 실행됨 — 사용자 입력 대기 중")

    def symptom_callback(self, msg: String):
        user_input = msg.data.strip()
        self.get_logger().info(f"[디버그] symptom_callback 호출됨: {user_input}")

        # 증상 입력 종료 신호
        if user_input == "__DONE__":
            try:
                path = os.path.expanduser("~/ros2_ws/src/pharmacy_main/resource/symptom_query.txt")
                with open(path, "r", encoding="utf-8") as f:
                    symptom_text = f.read().strip()
                self.get_logger().info(f"최종 증상 텍스트: {symptom_text}")
            except Exception as e:
                self.get_logger().error(f"symptom_query.txt 로딩 실패: {e}")
                return

            # 증상 → 약 리스트 요청
            recommended = self.call_get_medicine_name(symptom_text)
            self.get_logger().info(f"[디버그] 추천된 약 리스트: {recommended}")
            
            if not recommended:
                self.get_logger().info("약 추천 대기중...")
                return
            
<<<<<<< HEAD
            self.recommended_list = recommended
            # for drug in recommended:
            #     self.get_logger().info(f"추천된 약: {drug}")
            #     self.process_medicine(drug)

                # 결과를 GUI에도 보여주기 위해 전체 추천 결과 퍼블리시 (최초 한 번만)
            self.recommend_pub.publish(String(data=", ".join(recommended)))
            self.get_logger().info("[디버그] 약 리스트 저장됨. 첫 번째 약 처리 시작.")
            self.process_next_medicine()
=======
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
>>>>>>> 9de91ba720607219456fdaceac14b7017543d6d4
            return

        # 그 외 일반 텍스트 (참고용 로그)
        self.get_logger().info(f"(참고용) 입력 수신: \"{user_input}\"")

<<<<<<< HEAD
    def process_next_medicine(self, medicine_name: str):
        """리스트에서 다음 약 하나 꺼내서 감지 및 집기 진행"""
        if self.current_index >= len(self.recommended_list):
            self.get_logger().info("모든 약 처리 완료")
=======
    def process_medicine(self, medicine_name: str):

        
        result = self.call_detect_position(medicine_name)
        if not result:
            self.get_logger().error("약 위치 탐지 실패")
>>>>>>> 9de91ba720607219456fdaceac14b7017543d6d4
            return

        drug = self.recommended_list[self.current_index]
        self.get_logger().info(f"▶ 약 처리 시작: {drug}")

        retry_count = 0
        MAX_RETRY = 3

        while retry_count < MAX_RETRY:
            self.get_logger().info(f"[디버그] 감지 요청 시도: {drug}")
            result = self.call_detect_position(drug)
            if not result:
                self.get_logger().warn(f"{drug} 위치 탐지 실패 — 재시도 ({retry_count+1}/{MAX_RETRY})")
                retry_count += 1
                continue

            position, width = result
            point = Point()
            point.x, point.y, point.z = position


            self.get_logger().info(f"[디버그] 감지 성공: 좌표={position}, 폭={width}")
            self.get_logger().info(f"[디버그] pickup 요청 실행 중...")
            success = self.call_pickup([point], [width])
            self.get_logger().info(f"[디버그] pickup 응답: success={success}")
            if success:
                self.get_logger().info(f"{drug} 집기 성공")
                break
            else:
                self.get_logger().warn(f"{drug} 집기 실패 — 재시도 ({retry_count+1}/{MAX_RETRY})")
                retry_count += 1

        if retry_count >= MAX_RETRY:
            self.get_logger().error(f"{drug} 처리 실패 (최대 재시도 초과)")

        # 다음 약 처리 시작
        self.current_index += 1
        self.process_next_medicine()

    def call_get_medicine_name(self, symptom: str) -> list:
        if not self.get_medicine_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/get_medicine_name 서비스 연결 실패")
            return []

        request = GetMedicineName.Request()
        request.symptom = symptom
        future = self.get_medicine_client.call_async(request)
        
        timeout_sec = 15.0
        start_time = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                break
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error("서비스 응답 타임아웃! matcher가 응답을 안 줌")
                return []
        
        result = future.result()
        self.get_logger().info(f"[디버그] get_medicine 서비스 응답: {result}")

        if result and result.medicine:
            self.get_logger().info(f"추천된 약: {result.medicine}")
            return result.medicine
        else:
            self.get_logger().error("추천 결과 없음")
            return []

<<<<<<< HEAD
    def call_detect_position(self, medicine: str):
=======
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


                    # self.process_medicine(result.medicine)





                    print('뿅')
                else:
                    self.get_logger().info("약 정보 대기중")
            except Exception as e:
                self.get_logger().error(f"서비스 응답 처리 중 예외 발생: {e}")
        future.add_done_callback(response_callback)


        # rclpy.spin_until_future_complete(self, future)
        # return future.result().medicine if future.result() else None

    def call_detect_position(self, medicine):
>>>>>>> 9de91ba720607219456fdaceac14b7017543d6d4
        if not self.detect_position_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/get_3d_position 서비스 연결 실패")
            return None

        request = SrvDepthPosition.Request()
        request.target = medicine
        future = self.detect_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        self.get_logger().info(f"감지 결과 수신: {result}")
        if not result or sum(result.depth_position) == 0.0:
            self.get_logger().warn("[디버그] 감지 결과 없음")
            return None

        return result.depth_position, result.width

    def call_pickup(self, point: Point, width) -> bool:
        if not self.pickup_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/pickup_medicine 서비스 연결 실패")
            return False

        self.get_logger().info(f"[디버그] pickup 요청 전송: {point}, {width}")
        request = PickupMedicine.Request()
        request.point = point
        request.width = width
        future = self.pickup_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success if future.result() else False


def main(args=None):
    rclpy.init(args=args)
    node = PharmacyManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

