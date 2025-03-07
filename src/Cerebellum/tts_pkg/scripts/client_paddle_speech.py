#!/usr/bin/env python3
import rospy
import requests
import json
from tts_msgs.srv import TTS, TTSResponse


def handle_tts(req):
    try:
        # 解析输入 JSON
        request_data = json.loads(req.input_json)

        # 发送 HTTP 请求（假设服务端运行在容器内）
        response = requests.post(
            "http://localhost:8889/query", json=request_data, timeout=10
        )
        response.raise_for_status()

        # 返回 JSON 序列化结果
        return TTSResponse(json.dumps(response.json()))

    except requests.exceptions.RequestException as e:
        return TTSResponse(json.dumps({"error": f"HTTP Error: {str(e)}"}))
    except Exception as e:
        return TTSResponse(json.dumps({"error": f"Internal Error: {str(e)}"}))


def tts_server():
    rospy.init_node("paddle_speech_tts_server")
    rospy.Service("srv_tts", TTS, handle_tts)  # 服务名称  # 服务类型  # 回调函数
    rospy.loginfo("TTS Service Ready.")
    rospy.spin()


if __name__ == "__main__":
    tts_server()
