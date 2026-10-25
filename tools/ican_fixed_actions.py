"""固定动作目录：动作参数固化在M33，主机仅选择白名单ID。"""
LEGACY_PROFILE = "legacy-curl-j5-hold-j46-v5"
FIXED_PROFILE = "ican-fixed-j5-hold-j46-v6"
LVGL_PROFILE = "ican-lvgl-curl-j5-hold-j46-v7"
ACTIONS = {
    "elbow-full": {"label": "完整屈伸", "path": "/api/bench/5/curl", "opcode": 6,
                   "flexion_raw_deg": 84.14, "dwell_ms": 500},
    "elbow-small": {"label": "小幅屈伸", "path": "/api/bench/5/curl-small", "opcode": 12,
                    "flexion_raw_deg": 30, "dwell_ms": 500},
    "elbow-medium": {"label": "中幅屈伸", "path": "/api/bench/5/curl-medium", "opcode": 13,
                     "flexion_raw_deg": 60, "dwell_ms": 500},
    "elbow-pause": {"label": "屈曲停留再伸展", "path": "/api/bench/5/curl-pause", "opcode": 14,
                    "flexion_raw_deg": 60, "dwell_ms": 2000},
}


def supported_actions(profile):
    if profile == LEGACY_PROFILE:
        return ["elbow-full"]
    if profile in (FIXED_PROFILE, LVGL_PROFILE):
        return list(ACTIONS)
    return []


def binding_words(profile, mailbox):
    if profile == LEGACY_PROFILE:
        return [0x57444231, 9, mailbox, 64, 0x4c2708d2, 0x2f69d034, 0x431c5a72, 0x917be809]
    if profile == FIXED_PROFILE:
        return [0x57444231, 10, mailbox, 64, 0x734dca10, 0x2b897146, 0x69b3ed50, 0xa1c4f008]
    if profile == LVGL_PROFILE:
        return [0x57444231, 11, mailbox, 64, 0xe7c19011, 0x326a9d41, 0x5f21ab88, 0x9d643207]
    raise ValueError("未知动作固件配置")
