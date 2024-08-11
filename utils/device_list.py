# Script to list all available microphones and their respective index

import speech_recognition as sr
import pyaudio

def get_device_index(target_name):
    p = pyaudio.PyAudio()
    num_devices = p.get_device_count()
    
    for index in range(num_devices):
        device_info = p.get_device_info_by_index(index)
        device_name = device_info['name']
        
        if target_name in device_name:
            print(f"Using '{device_name}' for `Microphone(device_index={index})`")
            return index
    p.terminate()
    return None

if __name__ == '__main__':
    for index, name in enumerate(sr.Microphone.list_microphone_names()):
        print(f"Microphone with name \"{name}\" found for `Microphone(device_index={index})`")
