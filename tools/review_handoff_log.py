"""Read-only ULog handoff review. Never connects to a vehicle or changes a log."""
import argparse
import json
import math
import numpy as np
from pyulog import ULog


def scalar(value):
    if isinstance(value, np.generic):
        value = value.item()
    return None if isinstance(value, float) and not math.isfinite(value) else value


def review(path):
    log = ULog(path)
    origin = log.start_timestamp
    topics = {(d.name, d.multi_id): d.data for d in log.data_list}
    def at(name, usec, fields, instance=0):
        data = topics.get((name, instance))
        if data is None:
            return None
        index = int(np.searchsorted(data['timestamp'], usec, side='right')) - 1
        if index < 0:
            return None
        return {field: scalar(data[field][index]) for field in fields if field in data}
    def transitions(name, fields, instance=0):
        data = topics.get((name, instance), {})
        result, old = [], None
        for index, stamp in enumerate(data.get('timestamp', [])):
            values = {key: scalar(data[key][index]) for key in fields if key in data}
            if values != old:
                result.append({'t': round((int(stamp)-origin)/1e6, 3), **values})
                old = values
        return result
    sampled = []
    for sec in np.arange(0, (log.last_timestamp-origin)/1e6, 0.5):
        usec = origin + int(sec*1e6)
        primary = at('estimator_selector_status', usec, ['primary_instance'])
        instance = primary['primary_instance'] if primary else 0
        sampled.append({
            't': float(sec), 'primary': instance,
            'position': at('vehicle_local_position', usec, ['x','y','z','vx','vy','vz','heading','xy_valid','v_xy_valid']),
            'target': at('trajectory_setpoint', usec, ['x','y','z','vx','vy','vz','yaw','yawspeed']),
            'control': at('offboard_control_mode', usec, ['position','velocity']),
            'flow': at('optical_flow', usec, ['quality','sensor_id','integration_timespan']),
            'fusion': at('estimator_status_flags', usec, ['cs_opt_flow','cs_inertial_dead_reckoning','reject_optflow_x','reject_optflow_y','fs_bad_optflow_x','fs_bad_optflow_y'], instance),
            'ratios': at('estimator_innovation_test_ratios', usec, ['flow[0]','flow[1]'], instance),
            'pilot': at('manual_control_setpoint', usec, ['x','y','z','r','valid','data_source']),
            'mode': at('vehicle_status', usec, ['nav_state','arming_state']),
        })
    return {
        'firmware': {key: log.msg_info_dict.get(key) for key in ('ver_sw','ver_sw_branch','ver_sw_release','ver_hw')},
        'duration_s': (log.last_timestamp-origin)/1e6,
        'messages': [{'t': round((m.timestamp-origin)/1e6,3), 'text':m.message} for m in log.logged_messages],
        'control_transitions': transitions('offboard_control_mode',['position','velocity']),
        'mode_transitions': transitions('vehicle_status',['nav_state','arming_state']),
        'switch_transitions': transitions('manual_control_switches',['mode_slot','kill_switch','offboard_switch','return_switch']),
        'flow_transitions': transitions('optical_flow',['quality','sensor_id']),
        'primary_transitions': transitions('estimator_selector_status',['primary_instance','instance_changed_count']),
        'reset_transitions': transitions('vehicle_local_position',['xy_reset_counter','vxy_reset_counter','z_reset_counter','vz_reset_counter','heading_reset_counter']),
        'samples': sampled,
        'caveat': 'State fields are held from the preceding LOGGED sample; sparse ULog rates cannot prove exact event time or network sender identity. Not a live safety adapter.',
    }


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('log')
    p.add_argument('--output')
    args = p.parse_args()
    result = review(args.log)
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as stream:
            json.dump(result, stream, ensure_ascii=False, allow_nan=False, indent=2)
    else:
        print(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2))
