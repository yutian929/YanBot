import ChatTTS
import torch
import torchaudio
import os


class TTS:
    def __init__(self, random_voice=False, compile_model=False):
        """
        初始化TTS引擎

        Args:
            random_voice (bool): 是否随机生成音色，如果为False则使用default_voice.pt
            compile_model (bool): 是否编译模型以提高性能
        """
        # 加载模型
        self.chat = ChatTTS.Chat()
        self.chat.load(compile=compile_model)

        # 设置语音音色
        if random_voice:
            self.speaker = self.chat.sample_random_speaker()
            # 可选择保存随机生成的音色
            torch.save(self.speaker, "custom_voice.pt")
        else:
            default_voice_path = os.path.join(
                os.path.dirname(__file__), "default_voice.pt"
            )
            try:
                self.speaker = torch.load(default_voice_path)
                print(f"load default voice from {default_voice_path}")
            except FileNotFoundError:
                print("use random voice")
                self.speaker = self.chat.sample_random_speaker()
                # save as default voice for future use
                torch.save(self.speaker, default_voice_path)

    def text_to_speech(
        self, texts, output_file="output.wav", temperature=0.5, top_p=0.7, top_k=20
    ):
        """
        将文本转换为语音

        Args:
            texts (list): 要转换为语音的文本列表
            output_file (str): 输出的音频文件名
            temperature (float): 解码温度
            top_p (float): top P 解码参数
            top_k (int): top K 解码参数
        """
        params_infer_code = ChatTTS.Chat.InferCodeParams(
            spk_emb=self.speaker,
            temperature=temperature,
            top_P=top_p,
            top_K=top_k,
        )

        wavs = self.chat.infer(
            texts,
            params_infer_code=params_infer_code,
        )

        try:
            torchaudio.save(output_file, torch.from_numpy(wavs[0]).unsqueeze(0), 24000)
        except:
            torchaudio.save(output_file, torch.from_numpy(wavs[0]), 24000)


if __name__ == "__main__":
    tts_engine = TTS(random_voice=False, compile_model=False)
    tts_engine.text_to_speech(["你好, 我是燕燕.[uv_break] 你的护理机器人, 很高兴为你服务.[uv_break]"])
