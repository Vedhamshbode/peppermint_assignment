#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_service.srv import IdSw 
import sounddevice as sd
import librosa
import numpy as np
import wavio  # For saving as .wav
import time
from transformers import WhisperProcessor, WhisperForConditionalGeneration
from std_msgs.msg import String

processor = WhisperProcessor.from_pretrained("openai/whisper-small")
model = WhisperForConditionalGeneration.from_pretrained("openai/whisper-small")
model.config.forced_decoder_ids = None

# Set recording parameters
sample_rate = 16000  # Number of samples per second
duration = 5  # Duration in seconds

class VoiceService(Node):
    def __init__(self):
        super().__init__('server_node')
        self.srv = self.create_service(IdSw, 'process_voice', self.process_voice)
        self.get_logger().info("Service 'process_voice' is ready.")

    def process_voice(self, request, response):
        self.get_logger().info(f"Incoming request to record command: start={request.start}")
        if request.start:
            # Record audio using sounddevice
            print("Recording...")
            audio_data = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype='float32')
            sd.wait()  # Wait until recording is finished
            print('done')
            # Convert audio data to numpy array (mono, one-dimensional)
            audio_data = np.squeeze(audio_data)

            input_features = processor(audio_data, sampling_rate=sample_rate, return_tensors="pt").input_features 

            # generate token ids
            predicted_ids = model.generate(input_features)
            # decode token ids to text

            transcription = processor.batch_decode(predicted_ids, skip_special_tokens=True)
            print(transcription[0])
            # transcription_msg = String()
            # transcription_msg.data = transcription[0]


            response.data = transcription[0]
            response.res = True
        else :
            response.res = False
        return response

def main(args=None):
    rclpy.init(args=args)
    voice_service = VoiceService()
    rclpy.spin(voice_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
