import os
from eff_word_net.engine import HotwordDetector, MultiHotwordDetector
from eff_word_net.audio_processing import Resnet50_Arc_loss

class MHD:
    def __init__(self, command_ref, evaluate_ref, command_threshold=0.7, evaluate_threshold=0.71):
        base_model = Resnet50_Arc_loss()

        command_hw = HotwordDetector(
            hotword="command",
            model=base_model,
            reference_file=command_ref,
            threshold=command_threshold,
            relaxation_time=2,
        )

        evaluate_hw = HotwordDetector(
            hotword="evaluate",
            model=base_model,
            reference_file=evaluate_ref,
            threshold=evaluate_threshold,
            relaxation_time=2,
        )

        self.multi_hotword_detector = MultiHotwordDetector(
            [command_hw, evaluate_hw],
            model=base_model,
            continuous=True,
        )

    def find_best_match(self, frame):
        result = self.multi_hotword_detector.findBestMatch(frame)
        if None not in result:
            print(result[0], f",Confidence {result[1]:0.4f}")
            hotword = str(result[0]).split(': ')[1] if len(str(result[0]).split(': ')) > 1 else result[0]
            return hotword
        else:
            return None
