# credit to: https://github.com/Ant-Brain/EfficientWord-Net/blob/main/eff_word_net/streams.py

import pyaudio
from typing import Tuple , Callable
import numpy as np
from eff_word_net.engine import HotwordDetector


NoParameterFunction = Callable[[],None]
AudioFrameFunction = Callable[[],np.array]

RATE = 16000

class CustomAudioStream :
    """
    CustomAudioStream implementation allows developers to use 
    any 16000Hz sampled audio streams with inference engine

    It tries to add sliding window to audio streams
    """
    def __init__(
        self,
        open_stream:Callable[[],None],
        close_stream:Callable[[],None],
        get_next_frame:Callable[[],np.array],
        window_length_secs = 1,
        sliding_window_secs:float = 1/8
        ):

        self._open_stream = open_stream
        self._close_stream = close_stream
        self._get_next_frame = get_next_frame
        self._window_size = int(window_length_secs * RATE)
        self._sliding_window_size = int(sliding_window_secs * RATE)

        self._out_audio = np.zeros(self._window_size) #blank 1 sec audio
        print("Initial S",self._out_audio.shape)

    def start_stream(self):
        self._out_audio = np.zeros(self._window_size)
        self._open_stream()
        for i in range(RATE//self._sliding_window_size -1):
            self.getFrame()

    def close_stream(self):
        self._close_stream()
        self._out_audio = np.zeros(self._window_size)

    def getFrame(self):
        """
        Returns a 1 sec audio frame with sliding window of 1/8 sec with 
        sampling frequency 16000Hz
        """

        new_frame = self._get_next_frame()

        #print("Prior:", self._out_audio.shape, new_frame.shape )
        assert new_frame.shape == (self._sliding_window_size,), \
            "audio frame size from src doesnt match sliding_window_secs"


        self._out_audio = np.append(
                self._out_audio[self._sliding_window_size:],
            new_frame 
        )

        #print(self._out_audio.shape)

        return self._out_audio

class SimpleMicStream(CustomAudioStream) :

    """
    Implements mic stream with sliding window, 
    implemented by inheriting CustomAudioStream
    """
    def __init__(self,window_length_secs=1, sliding_window_secs:float=1/8):
        self._pyaudio_insteance = pyaudio.PyAudio()

        CHUNK = int(sliding_window_secs*RATE)
        print("Chunk size", CHUNK)
        self.mic_stream=self._pyaudio_insteance.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK
        )

        self.mic_stream.stop_stream()

        CustomAudioStream.__init__(
            self,
            open_stream = self.mic_stream.start_stream,
            close_stream = self.mic_stream.stop_stream,
            get_next_frame = lambda : (
                np.frombuffer(self.mic_stream.read(CHUNK,exception_on_overflow = False),dtype=np.int16) 
                ),
                 window_length_secs=window_length_secs,
                sliding_window_secs=sliding_window_secs
        )

    def free_audio_device(self):
        self._close_stream()
        self._pyaudio_insteance.terminate()