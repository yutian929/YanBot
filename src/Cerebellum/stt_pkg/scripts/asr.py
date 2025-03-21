from pathlib import Path
from funasr import AutoModel
from funasr.utils.postprocess_utils import rich_transcription_postprocess


class ASR:
    def __init__(self, model_dir="iic/SenseVoiceSmall", device="cuda:0"):
        self.model = AutoModel(
            model=model_dir,
            vad_model="fsmn-vad",
            vad_kwargs={"max_single_segment_time": 30000},
            device=device,
        )

    def transcribe(self, audio_path, language="auto"):
        """核心识别方法"""
        try:
            if not Path(audio_path).exists():
                return {"error": f"Audio file {audio_path} not found"}

            res = self.model.generate(
                input=str(audio_path),
                cache={},
                language=language,
                use_itn=True,
                batch_size_s=60,
                merge_vad=True,
                merge_length_s=15,
            )

            text = rich_transcription_postprocess(res[0]["text"])
            return text
        except Exception as e:
            return str(e)
