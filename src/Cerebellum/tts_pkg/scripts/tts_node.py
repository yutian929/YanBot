import rospy
import json
import os
from tts_pkg.srv import TTS, TTSResponse
from tts import TTS as TTS_ENGINE


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
        # 设置音频路径
        self.default_tts_wav_path = rospy.get_param("~default_tts_wav_path", "tts.wav")
        self.temperature = rospy.get_param("~temperature", 0.5)
        self.top_p = rospy.get_param("~top_p", 0.7)
        self.top_k = rospy.get_param("~top_k", 20)

        # 创建初始音频
        self.tts_engine.text_to_speech(
            texts=["你好, 我是燕燕, 很高兴为你服务. 请问我可以为你做什么嘛?[uv_break]"],
            output_file=self.default_tts_wav_path,
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
            if not text:
                raise ValueError("输入文本不能为空")

            # 前处理,加入[uv_break]
            text = text.replace(".", ".[uv_break] ")
            # 将所有的阿拉伯数字直接转换为中文数字
            convert_couple = {
                "1": "一",
                "2": "二",
                "3": "三",
                "4": "四",
                "5": "五",
                "6": "六",
                "7": "七",
                "8": "八",
                "9": "九",
                "0": "零",
            }
            text = "".join(
                [
                    convert_couple[char] if char in convert_couple else char
                    for char in text
                ]
            )

            # 生成语音
            self.tts_engine.text_to_speech(
                texts=[text],
                output_file=self.default_tts_wav_path,
                temperature=self.temperature,
                top_p=self.top_p,
                top_k=self.top_k,
            )

            return TTSResponse(
                output_json=json.dumps(
                    {"status": "success", "file_path": self.default_tts_wav_path}
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
