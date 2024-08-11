import os
import re
import subprocess
import argparse
import utils.baseline
import speech_recognition as sr
import google.generativeai as palm

from argparse import Namespace
from dbl_llm.dbl_llm import DBL
from utils.get_latency import ping
from utils.device_list import get_device_index
from utils.get_key_press import get_key_press
from utils.log_gen import record_timestamp, output_to_file
from utils.multi_hotword_detector import MHD
from utils.streams import SimpleMicStream
from utils.baseline import rule_based_action
from utils.get_driving_context import get_driving_context


# Uncomment to automatically set up the API key in the environment
# os.environ['OPENAI_API_KEY'] = ''


# Parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('-m', '--microphone_index', type=int, default=None, help='Index of the microphone device. Use utils/device_list.py to get the index.')
parser.add_argument('--microphone_name', type=str, default='USB Device', help='Microphone name if index is not provided.')
parser.add_argument('--llm_model_name', type=str, default='gpt-4', help='Model name for the LLM. See dbl_llm/dbl_llm.py for supported models.')
parser.add_argument('--template_name', type=str, default='parking', help='Template name for the LLM. Stored in templates/ directory.')
parser.add_argument('--memory_path', type=str, default='templates/memory_highway.txt', help='Path to the personalization memory module file.')
parser.add_argument('--enable_memory_input', type=bool, default=True, help='True to use memory module prompt.')
parser.add_argument('-c','--enable_driving_context', type=bool, default=True, help='Enable driving context in the input.')
parser.add_argument('-t1','--text_input_1', type=str, default=None, help='Preload up to 2 text inputs, the first input.')
parser.add_argument('-t2','--text_input_2', type=str, default=None, help='Preload up to 2 text inputs, the second input.')
parser.add_argument('-hwd','--hotword_detector', type=bool, default=True, help='Use hotword detector to trigger command or evaluation.')
parser.add_argument('-b', '--baseline', type=bool, default=False, help='Use baseline without LLM.')
args: Namespace = parser.parse_args()


def process_command(sr_recognizer, source):
    # Process text input
    if args.text_input_1 or args.text_input_2:
        mic_ready_time = record_timestamp("Microphone ready")
        recording_done_time = record_timestamp("Done recording command")
        if args.text_input_1:
            command = args.text_input_1
            args.text_input_1 = None
        else:
            command = args.text_input_2
            args.text_input_2 = None
        command_detected_time = record_timestamp("Done detecting command", recording_done_time)      
    else:    
    # Record voice input
        sr_recognizer.adjust_for_ambient_noise(source)
        mic_ready_time = record_timestamp("Microphone ready")
        audio = sr_recognizer.listen(source)
        recording_done_time = record_timestamp("Done recording command", mic_ready_time)
        # Audio to text
        try:
            command = sr_recognizer.recognize_whisper_api(audio, api_key=os.environ['OPENAI_API_KEY'])
        except sr.RequestError as e:
            print("Could not request results from Whisper API")
            return
        command_detected_time = record_timestamp("Done detecting command", recording_done_time)
    print(f"\"{command}\"")
    # Baseline w/o LLM
    if args.baseline:
        action = rule_based_action(args.template_name, command)
        print('Outputting action:',action)
        subprocess.Popen(action,shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        return command, action
    # Add Driving Context
    if args.enable_driving_context:
        driving_context = get_driving_context()
        print(f"Driving Context:\n{driving_context}")
        llm_input = driving_context+'\n'+command
    else:
        llm_input = command
    # Text to action with LLM
    action = DBL(llm_model_name=args.llm_model_name,
                template_name=args.template_name,
                memory_enable=args.enable_memory_input,
                memory_path=args.memory_path).run(llm_input)
    # clean up the action
    match = re.search(r'Action:\s*(.*)', action, flags=re.DOTALL)
    if match:
        action = match.group(1)
    action_output_time = record_timestamp("Done generating action", command_detected_time)
    print(f"\"{action}\"")
    # Execute action code
    subprocess.Popen(action,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE)
    action_executed_time = record_timestamp("Action executed successfully", action_output_time)
    # Output to log file
    output_to_file(detected_command=command,
                output_command=action,
                timestamps=[('Microphone Ready', mic_ready_time),
                            ('Done Recording Command', recording_done_time),
                            ('Done Detecting Command', command_detected_time),
                            ('Done Generating Action', action_output_time),
                            ('Action Executed Successfully', action_executed_time)])
    return command, action


def process_evaluation(command, action, sr_recognizer, source):
    # Only process evaluation if command and action are not empty
    try:
        assert len(command) > 0 and len(action) > 0
    except AssertionError:
        print("Command or Action is empty.")
        return

    # Record voice input
    sr_recognizer.adjust_for_ambient_noise(source)
    record_timestamp("Microphone ready")
    audio = sr_recognizer.listen(source)
    # Audio to text
    try:
        evaluation = sr_recognizer.recognize_whisper_api(audio, api_key=os.environ['OPENAI_API_KEY'])
    except sr.RequestError as e:
        print("Could not request results from Whisper API")
        return
    print("Detected Evaluation:\n", evaluation)
    # Save evaluation to MM
    with open(args.memory_path, 'a') as f:
        f.write(f'Command:\n{command}\nAction:\n{action}\nEvaluation:\n{evaluation}\n\n')
    record_timestamp("Written to memory module")


if __name__ == '__main__':
    # Test internet latency
    ping_s = ping('https://api.openai.com') * 1000
    print(f"Current ping to api.openai.com: {ping_s:.2f} ms")

    # Get microphone index if not provided
    if args.microphone_index is None:
        args.microphone_index = get_device_index(args.microphone_name)

    command = ''
    action = ''
    while True:
        if args.hotword_detector:
            # Initiate hotword detector
            mhd = MHD(command_ref='templates/hotword_command_ref.json',
                      evaluate_ref='templates/hotword_evaluate_ref.json')
            mic_stream = SimpleMicStream(window_length_secs=1.5, sliding_window_secs=0.75)
            mic_stream.start_stream()
            hotword = None
            iii = 0
            while hotword is None and iii < 10:
                frame = mic_stream.getFrame()
                hotword = mhd.find_best_match(frame)
                iii += 1
            mic_stream.free_audio_device()
            del mic_stream
            del mhd
            hotword = 'command'
            print(f"Detected hotword: {hotword}")
            # import time; time.sleep(5)
            # Process command or evaluation
            if hotword == 'command':
                print("Recording command...")
                sr_recognizer = sr.Recognizer()
                # print the name of the microphone using index using PyAudio\
                with sr.Microphone(device_index=args.microphone_index) as source:
                    command, action = process_command(sr_recognizer=sr_recognizer, source=source)
            elif hotword == 'evaluate':
                print("Recording evaluation...")
                sr_recognizer = sr.Recognizer()
                with sr.Microphone(device_index=args.microphone_index) as source:
                    process_evaluation(command=command, action=action, sr_recognizer=sr_recognizer, source=source)
                command, action = '', ''
        else:
            # Initialte new microphone
            sr_recognizer = sr.Recognizer()
            with sr.Microphone(device_index=args.microphone_index) as source:
                # Wait for key press
                print("Listening for key words...")
                key = get_key_press().lower()
                
                # Press 'c' or 'Enter' to command, 'e' to evaluate, 'q' to quit
                if key == 'c' or key == '\r' or key == '\n':  # '\r' or '\n' for Enter key
                    print("Recording command...")
                    command, action = process_command(sr_recognizer=sr_recognizer, source=source)
                    del sr_recognizer 
                elif key == 'e':
                    print("Recording evaluation...")
                    process_evaluation(command=command, action=action, sr_recognizer=sr_recognizer, source=source)
                    command, action = '', ''
                    del sr_recognizer
                elif key == 'q':
                    print("Quitting script...")
                    del sr_recognizer
                    break
