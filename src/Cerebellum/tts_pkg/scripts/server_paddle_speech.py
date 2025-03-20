# RUN in Docker
from flask import Flask, request, jsonify
from paddlespeech.cli.tts.infer import TTSExecutor
import os


tts = TTSExecutor()
app = Flask(__name__)


def get_normalized_path(relative_path):
    # 获取当前脚本的绝对路径
    current_script_path = os.path.abspath(__file__)
    print(f"current_script_path={current_script_path}")

    # 获取当前脚本所在目录
    current_dir = os.path.dirname(current_script_path)
    print(f"current_dir={current_dir}")

    # 拼接路径并规范化
    target_path = os.path.normpath(os.path.join(current_dir, relative_path))
    print(f"target_path={target_path}")
    # if not os.path.exists(target_path):
    #     raise FileNotFoundError(f"路径不存在：{target_path}")

    return target_path


@app.route("/query", methods=["POST"])
def handle_query():
    data = request.json
    query = data.get("query", "")
    print(f"Received query: {query}")
    file_name = "tts.wav"
    file = get_normalized_path(f"../../../../{file_name}")
    tts(text=query, output=file)
    return jsonify({"ans": file_name})


if __name__ == "__main__":
    # 预热
    file = get_normalized_path(f"../../../../tts.wav")
    tts(text="你好，我是雨田", output=file)
    print(f"预热完成,wav存放路径为:{file}")
    app.run(host="0.0.0.0", port=8889)  # 必须绑定到 0.0.0.0
