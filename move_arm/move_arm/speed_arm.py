# ファイル名 simple_client.py
import os
import sys # 端末から入力引数を受け取るために必要
import yaml
#from turtlesim.srv import Spawn
import rclpy
from rclpy.node import Node
from xarm_msgs.srv import MoveJoint
from xarm_msgs.srv import VacuumGripperCtrl
from ament_index_python.packages import get_package_share_directory

class StockNode(Node):

    def __init__(self):

        super().__init__('stocknode')

        self.exec_cli = self.create_client(MoveJoint, 'xarm/set_servo_angle') # クライアントの生成（型名、サービス名）
        while not self.exec_cli.wait_for_service(timeout_sec=1.0): # 1秒に1回サービスが利用できるかチェック
            self.get_logger().info('xarm_exec_plan service not available, waiting again...')
        self.exec_req = MoveJoint.Request()

        self.vacuum_cli = self.create_client(VacuumGripperCtrl, '/xarm/set_vacuum_gripper') # クライアントの生成（型名、サービス名）
        while not self.vacuum_cli.wait_for_service(timeout_sec=1.0): # 1秒に1回サービスが利用できるかチェック
            self.get_logger().info('set_vacuum_gripper service not available, waiting again...')
        self.open_vacuum_req = VacuumGripperCtrl.Request()
        self.close_vacuum_req = VacuumGripperCtrl.Request()
        self.open_vacuum_future = None
        self.close_vacuum_future = None
        
        package_name = 'move_arm'
        file_name = 'arm_position'
        save_path = os.path.join(
                    get_package_share_directory(package_name), 'config/')
        with open(save_path + file_name + '.yaml', 'r') as f:
            self.position = yaml.safe_load(f)
        print(self.position['init_position'])    

    def send_exec_request(self, position_name, speed=0.35, acc=10, mvtime=0):
        self.exec_req.angles = self.position[position_name]  #  亀の位置x座標
        self.exec_req.speed = float(speed)
        self.exec_req.acc = float(acc)
        self.exec_req.mvtime = float(mvtime)
        self.exec_future         = self.exec_cli.call_async(self.exec_req) # サービスをリクエストして、結果を非同期的に取得する

    def send_open_vacuum_request(self):
        self.open_vacuum_req.on           = True  #  亀の位置x
        self.open_vacuum_req.wait       = True
        self.open_vacuum_req.timeout      = 3.0
        self.open_vacuum_req.delay_sec     = 0.0
        self.open_vacuum_future         = self.vacuum_cli.call_async(self.open_vacuum_req) # サービスをリクエストして、結果を非同期的に取得する

    def send_close_vacuum_request(self):
        self.close_vacuum_req.on           = False  #  亀の位置x
        self.close_vacuum_req.wait       = True
        self.close_vacuum_req.timeout      = 3.0
        self.close_vacuum_req.delay_sec     = 0.0
        self.close_vacuum_future         = self.vacuum_cli.call_async(self.close_vacuum_req) # サービスをリクエストして、結果を非同期的に取得する

    def exec_wait(self):
        while rclpy.ok():
            rclpy.spin_once(self) # コールバック関数を1回だけ呼び出す
            if self.exec_future.done():  #  Futureがキャンセルされるか、結果を得るたらfuture.done（）がTrueになる。
                try:
                    response = self.exec_future.send_open_vacuum_request() .result() # サーバーから非同期的に送られてきたレスポンス
                    #response = self.exec_future.result()
                except Exception as e:                                         #  エラー時の処理
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:  #  エラーでないときは、端末にレスポンスである亀の名前を表示する
                    self.get_logger().info('Response:name=%s' % response)
                break      

    def open_vacuum_wait(self):
        while rclpy.ok():
            rclpy.spin_once(self) # コールバック関数を1回だけ呼び出す
            if self.open_vacuum_future.done():  #  Futureがキャンセルされるか、結果を得るたらfuture.done（）がTrueになる。
                try:
                    response = self.open_vacuum_future.result() # サーバーから非同期的に送られてきたレスポンス
                except Exception as e:                                         #  エラー時の処理
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:  #  エラーでないときは、端末にレスポンスである亀の名前を表示する
                    self.get_logger().info('Response:name=%s' % response)
                break

    def close_vacuum_wait(self):
        while rclpy.ok():
            rclpy.spin_once(self) # コールバック関数を1回だけ呼び出す
            if self.close_vacuum_future.done():  #  Futureがキャンセルされるか、結果を得るたらfuture.done（）がTrueになる。
                try:
                    response = self.close_vacuum_future.result() # サーバーから非同期的に送られてきたレスポンス
                except Exception as e:                                         #  エラー時の処理
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:  #  エラーでないときは、端末にレスポンスである亀の名前を表示する
                    self.get_logger().info('Response:name=%s' % response)
                break

def main(args=None):
    rclpy.init(args=args)
    simple_client = StockNode()     #  クライアンとオブジェクトの生成

#Phase 1
#    simple_client.send_plan_request('init_position')       # リクエストを送る
#    simple_client.plan_wait()
    
    simple_client.send_close_vacuum_request()
    simple_client.close_vacuum_wait()

    simple_client.send_exec_request('init_position')       # リクエストを送る
    simple_client.exec_wait()

    simple_client.send_exec_request('rice_ball_plum_pick', 2.8)       # リクエストを送る
    simple_client.exec_wait()

    simple_client.send_open_vacuum_request()
    simple_client.open_vacuum_wait()    
    
    simple_client.send_exec_request('rice_ball_plum')       # リクエストを送る
    simple_client.exec_wait()

    simple_client.send_exec_request('rice_ball_plum_pick', 2.8)       # リクエストを送る
    simple_client.exec_wait()

    simple_client.send_exec_request('init_position', 2.8)       # リクエストを送る
    simple_client.exec_wait()

    simple_client.send_close_vacuum_request()
    simple_client.close_vacuum_wait()

#    simple_client.send_plan_request('rice_ball_plum')       # リクエストを送る
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()       # リクエストを送る
#    simple_client.exec_wait()
#
#    
#
#    simple_client.send_plan_request('linear_plum_pick')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#
##    simple_client.send_close_vacuum_request() 
##    simple_client.close_vacuum_wait()
##
###Phase 2
##    simple_client.send_plan_request('init_position')       # リクエストを送る
##    simple_client.plan_wait()
##
##    simple_client.send_exec_request()       # リクエストを送る
##    simple_client.exec_wait()
##
##    simple_client.send_open_vacuum_request()
##    simple_client.open_vacuum_wait()
##
##    simple_client.send_plan_request('rice_ball_shake_pick')       # リクエストを送る
##    simple_client.plan_wait()
##    
##    simple_client.send_exec_request()
##    simple_client.exec_wait()   
##
##    simple_client.send_plan_request('rice_ball_shake')       # リクエストを送る
##    simple_client.plan_wait()
##
##    simple_client.send_exec_request()       # リクエストを送る
##    simple_client.exec_wait()
##
##    simple_client.send_plan_request('rice_ball_shake_pick')       # リクエストを送る
##    simple_client.plan_wait()
##
##    simple_client.send_exec_request()       # リクエストを送る
##    simple_client.exec_wait()
##
##    simple_client.send_close_vacuum_request() 
##    simple_client.close_vacuum_wait()
##
##
###Phase 3
##    simple_client.send_plan_request('init_position')       # リクエストを送る
##    simple_client.plan_wait()
##
##    simple_client.send_exec_request()       # リクエストを送る
##    simple_client.exec_wait()
##
##    simple_client.send_open_vacuum_request()
##    simple_client.open_vacuum_wait()
##
##    simple_client.send_plan_request('rice_ball_tuna_pick')       # リクエストを送る
##    simple_client.plan_wait()
##    
##    simple_client.send_exec_request()
##    simple_client.exec_wait()      
##
##    simple_client.send_plan_request('rice_ball_tuna')       # リクエストを送る
##    simple_client.plan_wait()
##
##    simple_client.send_exec_request()       # リクエストを送る
##    simple_client.exec_wait()
##
##    simple_client.send_plan_request('rice_ball_tuna_pick')       # リクエストを送る
##    simple_client.plan_wait()
##
##    simple_client.send_exec_request()       # リクエストを送る
##    simple_client.exec_wait()
##
##    simple_client.send_close_vacuum_request() 
##    simple_client.close_vacuum_wait()
##
##Phase 4
#    simple_client.send_plan_request('init_position')       # リクエストを送る
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()       # リクエストを送る
#    simple_client.exec_wait()
#
#    simple_client.send_open_vacuum_request()
#    simple_client.open_vacuum_wait()
#
#    simple_client.send_plan_request('sandwich_pick')       # リクエストを送る
#    simple_client.plan_wait()
#    
#    simple_client.send_exec_request()               
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('sandwich')       # リクエストを送る
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()       # リクエストを送る
#    simple_client.exec_wait()
# 
#    simple_client.send_plan_request('sandwich_pick')       # リクエストを送る
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()       # リクエストを送る
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('linear_sandwich')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('linear_sandwich2')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('sandwich_place')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('sandwich_pull')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#    simple_client.send_close_vacuum_request() 
#    simple_client.close_vacuum_wait()
#
#    simple_client.send_plan_request('sandwich_remove')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#   
#
###Phase 5
#    simple_client.send_plan_request('init_position')       # リクエストを送る
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()       # リクエストを送る
#    simple_client.exec_wait()
#
#    simple_client.send_open_vacuum_request()
#    simple_client.open_vacuum_wait()
#
#    simple_client.send_plan_request('pack_juice_pick')       # リクエストを送る
#    simple_client.plan_wait()
#    
#    simple_client.send_exec_request()
#    simple_client.exec_wait() 
#
#    simple_client.send_plan_request('pack_juice')       # リクエストを送る
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()       # リクエストを送る
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('pack_juice_pick')       # リクエストを送る
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()       # リクエストを送る
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('linear_pack_juice')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('linear_pack_juice2')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('rotate_pack_juice')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('pack_juice_place')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('pack_juice_place2')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('pack_juice_place3')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('pack_juice_place4')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('pack_juice_place5')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#    simple_client.send_close_vacuum_request() 
#    simple_client.close_vacuum_wait()
#
#    simple_client.send_plan_request('pack_juice_remove')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()
#
#    simple_client.send_plan_request('pack_juice_remove2')
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()
#    simple_client.exec_wait()    
#
#
##last_init
#    simple_client.send_plan_request('init_position')       # リクエストを送る
#    simple_client.plan_wait()
#
#    simple_client.send_exec_request()       # リクエストを送る
#    simple_client.exec_wait()

    simple_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
