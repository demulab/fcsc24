# ファイル名 simple_client.py
import sys # 端末から入力引数を受け取るために必要
from turtlesim.srv import Spawn
import rclpy
from rclpy.node import Node
from xarm_msgs.srv import PlanJoint
from xarm_msgs.srv import PlanExec
"""ros2 service call /xarm_exec_plan xarm_msgs/srv/PlanExec "wait: true"
"""
class RecognitionPoseNode(Node):

    def __init__(self):

        super().__init__('recognitionposenode')
        self.plan_cli = self.create_client(PlanJoint, 'xarm_joint_plan') # クライアントの生成（型名、サービス名）
        while not self.plan_cli.wait_for_service(timeout_sec=1.0): # 1秒に1回サービスが利用できるかチェック
            self.get_logger().info('service not available, waiting again...')
        self.plan_req = PlanJoint.Request() # リクエストインスタンスの生成。これにリクエストデータが格納される。

        self.exec_cli = self.create_client(PlanExec, 'xarm_exec_plan') # クライアントの生成（型名、サービス名）
        while not self.exec_cli.wait_for_service(timeout_sec=1.0): # 1秒に1回サービスが利用できるかチェック
            self.get_logger().info('service not available, waiting again...')
        self.exec_req = PlanExec.Request()

    def send_plan_request(self):
        self.plan_req.target           = [0.029052210971713066,-0.5858770608901978,-0.633608877658844,-2.9785525798797607,0.4072643220424652,-0.08881795406341553]  #  亀の位置x座標
        self.plan_future         = self.plan_cli.call_async(self.plan_req) # サービスをリクエストして、結果を非同期的に取得する

    def send_exec_request(self):
        self.exec_req.wait           = True  #  亀の位置x座標
        self.exec_future         = self.exec_cli.call_async(self.exec_req) # サービスをリクエストして、結果を非同期的に取得する


def main(args=None):
    rclpy.init(args=args)
    simple_client = RecognitionPoseNode()     #  クライアンとオブジェクトの生成  
    simple_client.send_plan_request()       # リクエストを送る

    while rclpy.ok():
        rclpy.spin_once(simple_client) # コールバック関数を1回だけ呼び出す
        if simple_client.plan_future.done():  #  Futureがキャンセルされるか、結果を得るたらfuture.done（）がTrueになる。
            try:
                response = simple_client.plan_future.result() # サーバーから非同期的に送られてきたレスポンス
            except Exception as e:                                         #  エラー時の処理
                simple_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:  #  エラーでないときは、端末にレスポンスである亀の名前を表示する
                simple_client.get_logger().info('Response:name=%s' % response)
            break

    simple_client.send_exec_request()       # リクエストを送る

    while rclpy.ok():
        rclpy.spin_once(simple_client) # コールバック関数を1回だけ呼び出す
        if simple_client.exec_future.done():  #  Futureがキャンセルされるか、結果を得るたらfuture.done（）がTrueになる。
            try:
                response = simple_client.exec_future.result() # サーバーから非同期的に送られてきたレスポンス
            except Exception as e:                                         #  エラー時の処理
                simple_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:  #  エラーでないときは、端末にレスポンスである亀の名前を表示する
                simple_client.get_logger().info('Response:name=%s' % response)
            break

    simple_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
