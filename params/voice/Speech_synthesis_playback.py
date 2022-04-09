#!/usr/bin/env python
# -*- coding: utf-8 -*-

from aip import AipSpeech
from playsound import playsound
import os

class Speech_synthesis_playback():
    def __init__(self):
        # 百度语音APP ID
		self.APP_ID = '17089547'
		self.API_KEY = 'b76MGFLcxqXZh6qqiFGs3OpC'
		self.SECRET_KEY = 'QWcAlCeWFzsGAZSYTgQFedRbcvxIYX12'
		self.client = AipSpeech(self.APP_ID, self.API_KEY, self.SECRET_KEY)
		self.ip = 'www.baidu.com'

		self.word = '为了你与他人的安全，请佩戴好口罩。'

		if os.system('ping -c 1 -w 1 %s'%self.ip):
			print '语音识别未通过，请重新输入！'
		else:
			result  = self.client.synthesis(self.word, 'zh', 2, {'vol': 4,})
			self.playSound(result)


    def playSound(self, res):
        # 识别正确返回语音二进制 错误则返回dict 参照下面错误码
        if not isinstance(res, dict):
            with open('auido.mp3', 'wb') as f:
                f.write(res)
        playsound("auido.mp3")


if __name__ == '__main__':
    Speech_synthesis_playback()
