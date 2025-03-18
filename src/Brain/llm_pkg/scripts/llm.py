from openai import OpenAI
import os
from typing import Generator, Optional, Dict, Any


class LLM:
    def __init__(
        self,
        model: str = "deepseek-ai/DeepSeek-V2.5",
        max_tokens: int = 4096,
        base_url: str = "https://api.siliconflow.cn/v1/",
        api_key: Optional[str] = None,
    ):
        self.model = model
        self.max_tokens = max_tokens
        self.base_url = base_url
        self.api_key = api_key or os.environ.get("SILICONFOLW_API_KEY")

        if not self.api_key:
            raise ValueError("未找到API密钥，请设置环境变量 SILICONFOLW_API_KEY 或通过参数传递")

        self.client = OpenAI(base_url=self.base_url, api_key=self.api_key)

    def _process_response(
        self, response: Any, stream: bool
    ) -> Generator[str, None, None]:
        if stream:
            # 流式响应处理
            first_content = True
            first_reason = True
            for chunk in response:
                delta = chunk.choices[0].delta

                if delta.content:
                    new_content = delta.content
                    if first_content:
                        yield "\n<ans>\n"
                        first_content = False
                    yield from new_content

                if hasattr(delta, "reasoning_content") and delta.reasoning_content:
                    new_reason = delta.reasoning_content
                    if first_reason:
                        yield "\n<think>\n"
                        first_reason = False
                    yield from new_reason
        else:
            # 非流式响应处理
            completion = response.choices[0].message
            if completion.content:
                yield f"\n<ans>\n{completion.content}"
            if (
                hasattr(completion, "reasoning_content")
                and completion.reasoning_content
            ):
                yield f"\n<think>\n{completion.reasoning_content}"

    def stream_chat(self, messages: list, **kwargs) -> Generator[str, None, None]:
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                stream=True,
                max_tokens=self.max_tokens,
                **kwargs,
            )
            return self._process_response(response, stream=True)
        except Exception as e:
            raise RuntimeError(f"API请求失败: {str(e)}") from e

    def non_stream_chat(self, messages: list, **kwargs) -> str:
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                stream=False,
                max_tokens=self.max_tokens,
                **kwargs,
            )
            # 直接传递响应对象，而不是列表包装
            return "".join(self._process_response(response, stream=False))
        except Exception as e:
            raise RuntimeError(f"API请求失败: {str(e)}") from e


# 使用示例
if __name__ == "__main__":
    llm = LLM()

    # 非流式对话示例
    print("非流式对话演示:")
    messages = [
        {"role": "system", "content": "你是一个AI助手，只能回答是或否。"},
        {"role": "user", "content": "你好。"},
    ]
    result = llm.non_stream_chat(messages)
    print(result)

    # 流式对话示例
    print("流式对话演示:")
    messages = [{"role": "user", "content": "你好。"}]
    for chunk in llm.stream_chat(messages):
        print(chunk, end="", flush=True)
