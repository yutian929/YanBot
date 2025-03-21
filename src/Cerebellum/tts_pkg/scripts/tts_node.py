import rospy
import json
import os
from tts_pkg.srv import TTS, TTSResponse
from tts import TTS as TTS_ENGINE


def get_normalized_path(relative_path):
    current_script_path = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_script_path)
    target_path = os.path.normpath(os.path.join(current_dir, relative_path))
    if not os.path.exists(target_path):
        os.makedirs(target_path)

    return target_path


class TTSNode:
    def __init__(self):
        rospy.init_node("tts_node")

        # 从ROS参数服务器获取配置
        self.random_voice = rospy.get_param("~random_voice", False)
        self.compile_model = rospy.get_param("~compile_model", False)

        # 初始化TTS引擎
        self.tts_engine = TTS_ENGINE(
            random_voice=self.random_voice, compile_model=self.compile_model
        )
        # 设置音频保存路径
        self.audio_relative_dir = "../../../../last_heard_audios"
        self.file_name = "tts.wav"
        self.file_path = os.path.join(
            get_normalized_path(self.audio_relative_dir), self.file_name
        )
        # 创建初始音频
        self.tts_engine.text_to_speech(
            texts=["你好, 我是燕燕, 很高兴为你服务. 请问我可以为你做什么嘛?[uv_break]"],
            output_file=self.file_path,
        )
        # 创建TTS服务
        self.service = rospy.Service("srv_tts", TTS, self.handle_tts_request)
        rospy.loginfo(f"TTS Service Ready (random_voice={self.random_voice})")
        rospy.loginfo("TTS Node Started")

    def handle_tts_request(self, req):
        """处理TTS请求"""
        try:
            # 解析输入JSON
            input_data = json.loads(req.input_json)
            text = input_data.get("text", "")

            # 验证输入有效性
            # if not text:
            #     raise ValueError("输入文本不能为空")

            # 前处理,加入[uv_break]
            text = text.replace(".", ".[uv_break] ")

            # 生成语音
            result_path = self.tts_engine.text_to_speech(
                texts=[text],
                output_file=self.file_path,
                temperature=input_data.get("temperature", 0.5),
                top_p=input_data.get("top_p", 0.7),
                top_k=input_data.get("top_k", 20),
            )

            return TTSResponse(
                output_json=json.dumps(
                    {"status": "success", "file_path": self.file_path}
                )
            )
        except Exception as e:
            return TTSResponse(
                output_json=json.dumps({"status": "error", "message": str(e)})
            )


if __name__ == "__main__":
    try:
        node = TTSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
