[temporarily deprecated]

"""
    脚本位于 xf_mic_asr_offline_circle/xf_mic.py
    需要修改的参数：
    1. 串口地址  port  可能是/dev/wheeltec_mic，我默认值写的/dev/ttyUSB0
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
