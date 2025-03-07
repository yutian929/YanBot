# import os
# from openai import OpenAI

# # 从环境变量获取API密钥
# api_key = os.environ.get("DEEPSEEK_API_KEY")
# if not api_key:
#     raise ValueError("请设置DEEPSEEK_API_KEY环境变量")

# # 创建客户端（只需要创建一次）
# client = OpenAI(api_key=api_key, base_url="https://api.deepseek.com")

# # 初始对话
# response = client.chat.completions.create(
#     model="deepseek-chat",
#     messages=[
#         {"role": "system", "content": "You are a helpful assistant"},
#         {"role": "user", "content": "Hello"},
#     ],
#     stream=False,
# )
# print(response.choices[0].message.content)

# # Round 1
# messages = [{"role": "user", "content": "9.11 and 9.8, which is greater?"}]
# response = client.chat.completions.create(model="deepseek-reasoner", messages=messages)

# reasoning_content = response.choices[0].message.reasoning_content
# content = response.choices[0].message.content
# print(reasoning_content)
# print(content)

# # Round 2
# messages.append({"role": "assistant", "content": content})
# messages.append(
#     {"role": "user", "content": "How many Rs are there in the word 'strawberry'?"}
# )
# response = client.chat.completions.create(model="deepseek-reasoner", messages=messages)
# # ...后续处理...

import ollama
from typing import List, Dict, Union


def ollama_chat(
    messages: List[Dict[str, str]],
    model: str = "deepseek-r1:8b",
    host: str = "localhost",
    port: str = "11434",
    **kwargs,
) -> Union[str, Dict]:
    """
    Ollama本地大模型对话封装函数

    参数：
    messages : 消息列表，格式示例：[{"role": "user", "content": "你好"}]
    model    : 模型名称（默认：deepseek-r1:8b）
    host     : 服务地址（默认：localhost）
    port     : 服务端口（默认：11434）
    **kwargs : 其他生成参数，支持：
               - temperature: 温度系数（默认0）
               - max_tokens: 最大生成token数
               - top_p: 核心采样概率
               - stream: 是否流式输出（默认False）

    返回：
    完整响应字典 或 错误信息字符串
    """
    try:
        # 初始化客户端
        client = ollama.Client(host=f"http://{host}:{port}")

        # 设置默认参数
        default_options = {"temperature": 0}
        if kwargs:
            default_options.update(kwargs)

        # 发送请求
        response = client.chat(model=model, messages=messages, options=default_options)

        # 返回完整响应或仅内容
        return (
            response if kwargs.get("full_response") else response["message"]["content"]
        )

    except ollama.ResponseError as e:
        return f"模型响应错误：{e.error}"
    except ConnectionError:
        return "连接失败，请检查Ollama服务是否运行"
    except Exception as e:
        return f"未知错误：{str(e)}"


# 使用示例
if __name__ == "__main__":
    import time

    st = time.time()

    # 基本用法
    messages = [{"role": "user", "content": "请用一句话介绍你自己"}]
    print(ollama_chat(messages))

    print(f"using {time.time() - st}")
    st = time.time()

    # 带参数的用法
    print(
        ollama_chat(
            messages, temperature=0.7, max_tokens=100, full_response=True  # 获取完整响应字典
        )
    )

    print(f"using {time.time() - st}")
