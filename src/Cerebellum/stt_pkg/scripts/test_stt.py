#!/usr/bin/env python3
import rospy
import json
from stt_pkg.srv import STT, STTRequest


def test_service():
    # 等待服务可用
    rospy.wait_for_service("/srv_stt")

    try:
        # 创建服务代理
        stt_proxy = rospy.ServiceProxy("/srv_stt", STT)
        stt_wav_path = rospy.get_param(
            "~default_stt_wav_path", "/home/yutian/YanBot/last_heard_audios/stt.wav"
        )

        # 构造请求
        request = STTRequest()
        request.input_json = f'{{"file_path": "{stt_wav_path}"}}'

        # 调用服务
        response = stt_proxy(request)
        print("Response:", json.loads(response.output_json))

    except rospy.ServiceException as e:
        print("Service call failed:", e)


if __name__ == "__main__":
    test_service()
