def have_word_in_list(word_list, sentence):
    for word in word_list:
        if word in sentence:
            return True
    return False

def rule_based_action(template_name, command):
    if template_name == 'highway':
        speed = 40
        lane = 'center'
        action = ''
        lookahead_dist = 18
        acc_word_list = ['speed up', 'accelerat', 'hurry','rush', 'quick', 'fast', ]
        dec_word_list = ['slow', 'deccelerat', 'brake', ]
        left_lane_word_list = ['left', 'overtake', 'over take', 'pass', ]
        right_lane_word_list = ['right',]
        if have_word_in_list(acc_word_list, command):
            speed = 50
        elif have_word_in_list(dec_word_list, command):
            speed = 30
        if have_word_in_list(left_lane_word_list, command):
            speed = 50
            lane = 'left'
        elif have_word_in_list(right_lane_word_list, command):
            speed = 30
            lane = 'right'
        if lane == 'left':
            return './scripts/lane_change_left.sh ' + str(speed) + ' ' + str(lookahead_dist)
        elif lane == 'right':
            return './scripts/lane_change_right.sh ' + str(speed) + ' ' + str(lookahead_dist)
        elif lane == 'center':
            return './scripts/speed_adjust.sh ' + str(speed) + ' ' + str(lookahead_dist)
        else:
            raise ValueError('Invalid Lane')
    elif template_name == 'intersection':
        speed = 20
        lookahead_dist = 12
        act = 'stop'
        action = ''
        acc_word_list = ['speed up', 'accelerat', 'hurry', 'rush', 'quick', 'fast', ]
        dec_word_list = ['slow', 'deccelerat', 'brake', ]
        turn_left_word_list = ['turn', 'left','go ', ]
        stop_word_list = ['stop', 'wait', 'brake', ]
        if have_word_in_list(acc_word_list, command):
            speed = 30
            lookahead_dist = 18
        elif have_word_in_list(dec_word_list, command):
            speed = 15
            lookahead_dist = 12
        if have_word_in_list(turn_left_word_list, command):
            act = 'turn'
        elif have_word_in_list(stop_word_list, command):
            act = 'stop'
        if act == 'stop':
            return './scripts/stop.sh ' + str(speed) + ' ' + str(lookahead_dist)
        elif act == 'turn':
            return './scripts/turn_left.sh '+ str(speed) + ' ' + str(lookahead_dist)
        else:
            return './scripts/stop.sh 20 12'
    elif template_name == 'parking':
        nearest_word_list = ['nearest','fast','quick','hurry','rush','as soon as possible',]
        statium_word_list = ['statium',]
        exit_word_list = ['exit','leave','go out','out',]
        action = ''
        if have_word_in_list(nearest_word_list, command):
            return './scripts/nearest.sh'
        elif have_word_in_list(statium_word_list, command):
            return './scripts/statium.sh'
        elif have_word_in_list(exit_word_list, command):
            return './scripts/exit.sh'
        else:
            return './scripts/nearest.sh'