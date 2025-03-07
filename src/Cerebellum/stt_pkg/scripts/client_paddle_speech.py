#!/usr/bin/env python3
import rospy
import requests
import json
from stt_msgs.srv import STT, STTResponse


def handle_stt(req):
    try:
        # 解析输入 JSON
        request_data = json.loads(req.input_json)

        # 发送 HTTP 请求（假设服务端运行在容器内）
        response = requests.post(
            "http://localhost:8888/query", json=request_data, timeout=10
        )
        response.raise_for_status()

        # 返回 JSON 序列化结果
        return STTResponse(json.dumps(response.json()))

    except requests.exceptions.RequestException as e:
        return STTResponse(json.dumps({"error": f"HTTP Error: {str(e)}"}))
    except Exception as e:
        return STTResponse(json.dumps({"error": f"Internal Error: {str(e)}"}))


def stt_server():
    rospy.init_node("paddle_speech_stt_server")
    rospy.Service("srv_stt", STT, handle_stt)  # 服务名称  # 服务类型  # 回调函数
    rospy.loginfo("STT Service Ready.")
    rospy.spin()


if __name__ == "__main__":
    stt_server()
