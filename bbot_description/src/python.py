import sounddevice as sd
import librosa
import numpy as np
import wavio  # For saving as .wav
import time
from transformers import WhisperProcessor, WhisperForConditionalGeneration
# load model and processor
processor = WhisperProcessor.from_pretrained("openai/whisper-small")
model = WhisperForConditionalGeneration.from_pretrained("openai/whisper-small")
model.config.forced_decoder_ids = None


# Set recording parameters
sample_rate = 16000  # Number of samples per second
duration = 5  # Duration in seconds

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