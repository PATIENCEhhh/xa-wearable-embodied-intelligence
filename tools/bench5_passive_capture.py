"""只读采集5号位置/速度，用于停用状态反馈一致性排查；不授权、不使能、不发目标。"""
import argparse
import json
import math
from pathlib import Path
import time

from wifi_arm_debug import query


def summarize(rows):
    """使用板端回复时间计算位置差分；位置和速度并非同时采样。"""
    positions = [r for r in rows if r.get('ok') and r['operation'] == 'position']
    velocities = [r['value_raw'] for r in rows if r.get('ok') and r['operation'] == 'velocity']
    rates = []
    for a, b in zip(positions, positions[1:]):
        dt = (b['reply_tick'] - a['reply_tick']) & 0xffffffff
        if 0 < dt < 10000:
            rates.append((b['value_raw'] - a['value_raw']) * 1000 / dt)
    return {
        'position_samples': len(positions), 'velocity_samples': len(velocities),
        'failed_samples': sum(not r.get('ok') for r in rows),
        'position_span_rad': max(r['value_raw'] for r in positions) - min(r['value_raw'] for r in positions) if positions else None,
        'velocity_mean_rad_s': sum(velocities) / len(velocities) if velocities else None,
        'velocity_peak_abs_rad_s': max(map(abs, velocities)) if velocities else None,
        'position_rate_peak_abs_rad_s': max(map(abs, rates)) if rates else None,
        'position_rates_rad_s': rates,
        'limitation': '异步、稀疏采样仅用于诊断；不能证明物理停止，也不能排除采样间运动。',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.5.182')
    parser.add_argument('--pairs', type=int, default=15)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.pairs <= 120:
        parser.error('采样对数必须为1～120')
    rows = []
    # 排除已有动作，不改变运行状态；不占用活动试动的参数缓存。
    bench = query(args.host, 'bench', None)
    if not bench.get('ok') or bench.get('active'):
        raise RuntimeError(f'控制状态不明或动作尚未结束，拒绝采集：{bench}')
    time.sleep(0.5)
    try:
        for _ in range(args.pairs):
            for operation in ('position', 'velocity'):
                result = query(args.host, operation, 5)
                result['operation'] = operation
                result['host_monotonic'] = time.monotonic()
                if result.get('ok') and (result.get('motor_id') != 5 or
                        result.get('index') != (0x7019 if operation == 'position' else 0x701b) or
                        not math.isfinite(result['value_raw'])):
                    raise RuntimeError('回复来源、参数索引或数值异常')
                rows.append(result)
                print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
                if not result.get('ok'):
                    raise RuntimeError('电机未回复，已保存现有数据；不会发送动作')
                time.sleep(0.5)
    finally:
        args.output.write_text(json.dumps({'initial_bench': bench, 'samples': rows,
                              'summary': summarize(rows)}, ensure_ascii=False, indent=2,
                              allow_nan=False), encoding='utf-8')
    print(json.dumps(summarize(rows), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
