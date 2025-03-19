#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Int32, Bool
import time
import serial
import signal
import json
import re
import wave
from pyaudio import PyAudio, paInt16
import numpy as np
import pyudev


def find_device_port(vendor_id, product_id, try_count=10):
    for i in range(try_count):
        context = pyudev.Context()
        for device in context.list_devices(subsystem="tty"):
            if (
                device.get("ID_VENDOR_ID") == vendor_id
                and device.get("ID_MODEL_ID") == product_id
            ):
                return device.device_node
        time.sleep(1)
        print(f"Device not found, retrying... ({i+1}/{try_count})")
    return None


"""

    需要修改的参数：
    1. 串口地址  port  可能是/dev/wheeltec_mic
    2. 唤醒词    xf_mic_asr_offline_circle.set_wakeup_words  在python脚本最后，现在写的是“你好燕燕“
        唤醒词需要拆成拼音 如：“你好燕燕”  拆成 “ni3 hao3 yan4 yan4”
    3. 唤醒词识别阈值 python脚本最后，也在唤醒词那个函数里，范围0-1000，默认值300

    话题：
    1. 唤醒词识别声源角度  /mic/angle Int32
    2. 唤醒词识别状态  /mic/awake    Bool

    可能存在问题的部分：
    1. 录音
        Audio_Record 类里的录音参数，可能需要修改，如采样率，声道数等，还有录音设备，
                     如果录音设备只有讯飞这一个输入设备，可能不需要修改
    依赖：
    1. 安装pyaudio  pip3 install pyaudio
    2. 安装pyserial  pip3 install pyserial
    3. 安装wave  pip3 install wave
        In one go: pip3 install pyaudio pyserial wave

    运行：
        rosrun xf_mic_asr_offline_circle xf_mic.py
        其他的不用开(理论上)

        可能需要的操作：
            roscd xf_mic_asr_offline_circle
            chmod +x xf_mic.py

 """


awake = False
port = "/dev/wheeltec_mic"
if port is None:
    raise Exception("Device not found!")


class xf_mic_asr_offline_circle:
    def __init__(self):
        self.ser = serial.Serial(
            port,
            115200,
            serial.EIGHTBITS,
            serial.PARITY_NONE,
            serial.STOPBITS_ONE,
            timeout=0.05,
        )
        self.key = r"{\"content.*?aiui_event\"}"
        self.key_type = r"{\"code.*?\"}"
        self.running = True

    def send(self, header, param):
        result = ""
        while 1:
            count = self.ser.inWaiting()
            if count != 0:
                recv = self.ser.readall()
                if (
                    recv[0] == 0xA5
                    and recv[1] == 0x01
                    and (recv[2] == 0xFF or recv[2] == 0x01)
                ):

                    # 收到设备握手请求
                    msg_id = recv[5] + (recv[6] << 8)
                    ack_msg = [
                        0xA5,
                        0x01,
                        0xFF,
                        0x04,
                        0x00,
                        0x00,
                        0x00,
                        0xA5,
                        0x00,
                        0x00,
                        0x00,
                    ]
                    ack_msg[5] = msg_id & 0xFF
                    ack_msg[6] = msg_id >> 8
                    checksum = 255 - (sum(ack_msg) % 256) + 1
                    ack_msg.append(checksum)
                    self.ser.write(ack_msg)
                    time.sleep(0.1)

                    # 老的逻辑
                    # self.ser.write([0xa5, 0x01, 0xff, 0x04, 0x00, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0xb2])

                    data = bytes(json.dumps(param), encoding="utf-8")
                    length = len(data)
                    header.extend([length & 0xFF, length >> 8])
                    header.extend([0x00, 0x00])
                    header.extend(data)
                    checksum = 255 - (sum(header) % 256) + 1
                    header.extend([checksum])
                    self.ser.write(header)
                    break
            else:
                self.ser.write(
                    [
                        0xA5,
                        0x01,
                        0x01,
                        0x04,
                        0x00,
                        0x00,
                        0x00,
                        0xA5,
                        0x00,
                        0x00,
                        0x00,
                        0xB0,
                    ]
                )
                time.sleep(0.1)

        while 1:
            count = self.ser.inWaiting()
            if count != 0:
                recv = self.ser.readall()
                if (
                    recv[0] == 0xA5
                    and recv[1] == 0x01
                    and (recv[2] == 0xFF or recv[2] == 0x01)
                ):
                    result = recv
                    msg_id = recv[5] + (recv[6] << 8)
                    ack_msg = [
                        0xA5,
                        0x01,
                        0xFF,
                        0x04,
                        0x00,
                        0x00,
                        0x00,
                        0xA5,
                        0x00,
                        0x00,
                        0x00,
                    ]
                    ack_msg[5] = msg_id & 0xFF
                    ack_msg[6] = msg_id >> 8
                    checksum = 255 - (sum(ack_msg) % 256) + 1
                    ack_msg.append(checksum)
                    self.ser.write(ack_msg)

                    # 老的逻辑

                    # self.ser.write([0xa5,0x01,0xff,0x04,0x00,0x00,0x00,0xa5,0x00,0x00,0x00,0xb2])
                    break
        return result

    def set_wakeup_words(self, pinyin="ni3 hao3 yan4 yan4", threshold=300):
        param = {
            "type": "wakeup_keywords",
            "content": {
                "keywords": pinyin,
                "threshold": threshold,
            },
        }
        header = [0xA5, 0x01, 0x05]
        self.send(header, param)

    def get_awake_status(self):
        global angle_pub, awake_pub, awake, AudioRecord
        while self.running:
            count = self.ser.inWaiting()
            if count != 0:
                recv_data = self.ser.readall()
                if b"content" in recv_data:
                    pattern = re.compile(self.key)
                    m = re.search(pattern, str(recv_data).replace("\\", ""))
                    if m is not None:
                        m = m.group(0).replace('"{"', '{"').replace('}"', "}")
                        if m is not None:
                            try:
                                data = json.loads(m)
                                angle = data["content"]["info"]["ivw"]["angle"]
                            except KeyError as e:
                                rospy.logerr(f"JSON data missing key: {e}")
                                continue
                            awake_pub.publish(True)
                            angle_pub.publish(angle)
                            awake = True
                            rospy.loginfo(f"Wake up angle: {angle}")
                            AudioRecord.read_audio()
                            AudioRecord.save_wav("temp_voice.wav")
                            awake = False
            else:
                self.ser.write(
                    [
                        0xA5,
                        0x01,
                        0x01,
                        0x04,
                        0x00,
                        0x00,
                        0x00,
                        0xA5,
                        0x00,
                        0x00,
                        0x00,
                        0xB0,
                    ]
                )
                time.sleep(0.1)

    def get_version(self):
        param = {"type": "version"}
        header = [0xA5, 0x01, 0x05]
        result = self.send(header, param)
        if result is not None:
            pattern = re.compile(self.key_type)
            msg = re.search(pattern, str(result).replace("\\", ""))
            if msg is not None:
                return msg.group(0)
        else:
            return False


class Audio_Record(object):
    def __init__(self, seconds=10):
        self.num_samples = 8000  # pyaudio内置缓冲大小
        self.sampling_rate = 16000  # 取样频率
        self.level = 1500  # 声音保存的阈值
        self.count_num = 20  # count_num个取样之内出现COUNT_NUM个大于LEVEL的取样则记录声音
        self.save_length = 3  # 声音记录的最小长度：save_length * num_samples 个取样
        self.time_count = seconds  # 录音时间，单位s
        self.voice_string = []

    # 保存文件
    def save_wav(self, filename):
        wf = wave.open(filename, "wb")
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(self.sampling_rate)
        wf.writeframes(np.array(self.voice_string).tostring())
        wf.close()

    def read_audio(self):
        pa = PyAudio()
        stream = pa.open(
            format=paInt16,
            channels=1,
            rate=self.sampling_rate,
            input=True,
            frames_per_buffer=self.num_samples,
        )

        save_count = 0
        save_buffer = []
        time_count = self.time_count
        start_rec = 0
        while True:
            if start_rec == 2:
                time_count -= 1

            # 读入num_samples个取样
            string_audio_data = stream.read(self.num_samples)
            # 将读入的数据转换为数组
            audio_data = np.fromstring(string_audio_data, dtype=np.short)
            # 计算大于 level 的取样的个数
            large_sample_count = np.sum(audio_data > self.level)

            print(np.max(audio_data)), "large_sample_count=>", large_sample_count

            # 如果个数大于COUNT_NUM，则至少保存SAVE_LENGTH个块
            if large_sample_count > self.count_num:
                save_count = self.save_length
                start_rec = 1
            else:
                if start_rec == 1:
                    save_count -= 1

            if start_rec == 1:
                time_count -= 1
            if save_count < 0:
                save_count = 0

            if save_count > 0:
                save_buffer.append(string_audio_data)
            else:
                if len(save_buffer) > 0:
                    self.voice_string = save_buffer
                    save_buffer = []
                    print("Record finished.")
                    stream.close()
                    pa.terminate()
                    return True

            if time_count == 0:  # 时间到
                if len(save_buffer) > 0:
                    self.voice_string = save_buffer
                    save_buffer = []
                    print("Record finished.")
                    stream.close()
                    pa.terminate()
                    return True
                else:
                    stream.close()
                    pa.terminate()
                    return False


if __name__ == "__main__":
    rospy.init_node("xf_mic_asr_offline_circle")
    xf_mic_asr_offline_circle = xf_mic_asr_offline_circle()
    AudioRecord = Audio_Record()
    xf_mic_asr_offline_circle.set_wakeup_words("ni3 hao3 yan4 yan4", 300)
    version = xf_mic_asr_offline_circle.get_version()
    if version is not False:
        rospy.loginfo(f"Xf_mic_asr_offline_circle version: {version}")
    angle_pub = rospy.Publisher("/mic/angle", Int32, queue_size=10)
    awake_pub = rospy.Publisher("/mic/awake", Bool, queue_size=10)
    while not rospy.is_shutdown():
        xf_mic_asr_offline_circle.get_awake_status()
        rospy.spin()
