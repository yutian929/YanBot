#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from llm import LLM
from llm_pkg.srv import LLMChat, LLMChatResponse
import json
import os


class LLMNode:
    def __init__(self):
        rospy.init_node("llm_service_node", anonymous=True)

        # 初始化LLM实例
        self.chat_llm = LLM("deepseek-ai/DeepSeek-V2.5")
        self.reason_llm = LLM("deepseek-ai/DeepSeek-R1")
        # 问答类型
        self.system_types = json.load(
            open(os.path.join(os.path.dirname(__file__), "system_types.json"), "r")
        )
        self.chat_types = self.system_types["chat"].keys()
        self.reason_types = self.system_types["reason"].keys()
        # 创建服务
        self.service = rospy.Service(
            "service_llm_chat", LLMChat, self.handle_llm_chat_request
        )
        self.service_reason = rospy.Service(
            "service_llm_reason", LLMChat, self.handle_llm_reason_request
        )

        rospy.loginfo("LLM服务节点已启动")

    def handle_llm_chat_request(self, req):
        try:
            type = req.type
            if type not in self.chat_types:
                rospy.logwarn(f"不支持的问答类型: {type}")
                type = "simple_chat"
            # 构建消息列表
            messages = [
                {
                    "role": "system",
                    "content": self.system_types["chat"][type]["system_prompt"],
                },
                {"role": "user", "content": req.content},
            ]

            # 使用非流式响应
            response = self.chat_llm.non_stream_chat(messages)

            return LLMChatResponse(success=True, response=str(response))

        except Exception as e:
            rospy.logerr(f"处理LLM Chat请求时发生错误: {str(e)}")
            return LLMChatResponse(success=False, response=str(e))

    def handle_llm_reason_request(self, req):
        try:
            type = req.type
            if type not in self.reason_types:
                rospy.logwarn(f"不支持的推理类型: {type}")
                type = "simple_reason"
            # 构建消息列表
            messages = [
                {
                    "role": "system",
                    "content": self.system_types["reason"][type]["system_prompt"],
                },
                {"role": "user", "content": req.content},
            ]

            response = self.reason_llm.non_stream_chat(messages)

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
