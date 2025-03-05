from transformers import WhisperProcessor, WhisperForConditionalGeneration
import librosa 

# load model and processor
processor = WhisperProcessor.from_pretrained("openai/whisper-small")
model = WhisperForConditionalGeneration.from_pretrained("openai/whisper-small")
model.config.forced_decoder_ids = None


sample = librosa.load('output_back.wav', sr=16000)
print(sample)
input_features = processor(sample[0], sampling_rate=sample[1], return_tensors="pt").input_features 

# generate token ids
predicted_ids = model.generate(input_features)
# decode token ids to text

transcription = processor.batch_decode(predicted_ids, skip_special_tokens=True)
print(transcription[0])
