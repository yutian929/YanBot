import rospy
import json
import os
from stt_pkg.srv import STT, STTResponse
from asr import ASR


def get_normalized_path(relative_path):
    current_script_path = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_script_path)
    target_path = os.path.normpath(os.path.join(current_dir, relative_path))

    if not os.path.exists(target_path):
        raise FileNotFoundError(f"路径不存在：{target_path}")

    return target_path


class STTNode:
    def __init__(self):
        rospy.init_node("stt_node")
        # 初始化ASR处理器
        model_dir = rospy.get_param("~model_dir", "iic/SenseVoiceSmall")
        self.asr = ASR(model_dir)
        self.audio_relative_dir = "../../../../last_heard_audios"
        # 创建服务
        self.service = rospy.Service("srv_stt", STT, self.handle_stt_request)
        rospy.loginfo("STT Service Ready")
        rospy.loginfo(f"stt_node initialized with model_dir={model_dir}")

    def handle_stt_request(self, req):
        """处理服务请求"""
        try:
            # 解析输入JSON
            input_data = json.loads(req.input_json)
            audio_name = input_data.get("file_name", "stt.wav")
            audio_path = get_normalized_path(f"{self.audio_relative_dir}/{audio_name}")

            # 执行语音识别
            result = self.asr.transcribe(audio_path)

            # 构造响应
            return STTResponse(output_json=json.dumps(result))
        except Exception as e:
            return STTResponse(output_json=json.dumps({"error": str(e)}))


if __name__ == "__main__":
    try:
        node = STTNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
