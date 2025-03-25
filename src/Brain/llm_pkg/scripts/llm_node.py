#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from llm import LLM
from llm_pkg.srv import LLMChat, LLMChatResponse
import json
import os


class LLMNode:
    def __init__(self):
        rospy.init_node("llm_node")

        # 初始化LLM实例
        self.chat_llm = LLM("deepseek-ai/DeepSeek-V2.5")
        self.reason_llm = LLM("deepseek-ai/DeepSeek-R1")
        # 问答类型
        default_llm_types_path = os.path.join(
            os.path.dirname(__file__), "llm_types.json"
        )
        llm_types_path = rospy.get_param("~llm_types_path", default_llm_types_path)
        self.llm_types = json.load(open(llm_types_path, "r"))
        self.chat_types = self.llm_types["chat"].keys()
        self.reason_types = self.llm_types["reason"].keys()
        # 创建服务
        service_llm_chat = rospy.get_param("~service_llm_chat", "llm_chat")
        self.service = rospy.Service(
            service_llm_chat, LLMChat, self.handle_llm_chat_request
        )
        service_llm_reason = rospy.get_param("~service_llm_reason", "llm_reason")
        self.service_reason = rospy.Service(
            service_llm_reason, LLMChat, self.handle_llm_reason_request
        )

        rospy.loginfo("llm_node initialized complete.")

    def _process_response(self, response):
        response = response.strip()
        response = response.replace("<ans>", "").replace("<think>", "")
        response = response.replace("\n", "")
        return response

    def handle_llm_chat_request(self, req):
        try:
            type = req.type
            if type not in self.chat_types:
                rospy.logwarn(f"不支持的问答类型: {type}")
                type = "test_chat"
            # 构建消息列表
            messages = [
                {
                    "role": "system",
                    "content": self.llm_types["chat"][type]["system_prompt"],
                },
                {"role": "user", "content": req.content},
            ]

            # 使用非流式响应
            response = self.chat_llm.non_stream_chat(messages)
            response = self._process_response(response)
            return LLMChatResponse(success=True, response=str(response))

        except Exception as e:
            rospy.logerr(f"处理LLM Chat请求时发生错误: {str(e)}")
            return LLMChatResponse(success=False, response=str(e))

    def handle_llm_reason_request(self, req):
        try:
            type = req.type
            if type not in self.reason_types:
                rospy.logwarn(f"不支持的推理类型: {type}")
                type = "test_reason"
            # 构建消息列表
            messages = [
                {
                    "role": "system",
                    "content": self.llm_types["reason"][type]["system_prompt"],
                },
                {"role": "user", "content": req.content},
            ]

            response = self.reason_llm.non_stream_chat(messages)
            response = self._process_response(response)
            return LLMChatResponse(success=True, response=str(response))

        except Exception as e:
            rospy.logerr(f"处理LLM Reason请求时发生错误: {str(e)}")
            return LLMChatResponse(success=False, response=str(e))


if __name__ == "__main__":
    try:
        llm_node = LLMNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
