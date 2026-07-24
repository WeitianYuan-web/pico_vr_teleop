import os
import sys
import time
import zenoh
from io_bus_proto.io_bus_codec import decode_tf_message

zenoh_cf=zenoh.Config.from_file('./configs/config/zenoh.json5')

# 透传同时间戳的exo出到align的耗时
def cal_delay(hand_name, t=5):
    esk_key = "io_fusion/tf_exoskeleton"
    align_key = f"io_align/{hand_name}/tf_hand"
    esk_time = {}  # stamp_ns -> 收到 esk 的时间
    latencies = []  # 花费时间列表--计算平均

    # 订阅exo和align的回调
    def on_msg(sample):
        # 收到时间戳
        recv_ns = time.time_ns()
        # 收到时间戳对应的数据key
        key = str(sample.key_expr) 
        stamp_ns = decode_tf_message(sample.payload.to_bytes())["stamp_ns"]
         
        # 如果收到exo，添加到字典记录时间戳和收到时间
        if key == esk_key:
            esk_time[stamp_ns] = recv_ns
        elif key == align_key and stamp_ns in esk_time:
            # 计算花费时间
            ms = (recv_ns - esk_time.pop(stamp_ns)) / 1e6
            latencies.append(ms)  # 添加到花费时间列表ms
            
            print(f"stamp={stamp_ns}  delay={ms:.2f} ms")

    print(f"订阅 {esk_key} + {align_key}，测 {t}s\n")
    with zenoh.open(zenoh_cf) as session:
        session.declare_subscriber(esk_key, on_msg)
        session.declare_subscriber(align_key, on_msg)
        # 测试t秒
        time.sleep(t)

    return latencies


if __name__ == "__main__":
    # 灵巧手名称
    hand_name = "DexcelRobotics_Apex"
    
    try:
        latencies = cal_delay(hand_name, 10)
        if latencies:
            print(f"\n共 {len(latencies)} 帧，平均耗时 {sum(latencies) / len(latencies):.2f} ms")
        else:
            print("未收到匹配帧")
    except Exception as e:
        print(f"error: {e}")
    finally:
        print("测试结束")
        sys.exit(0)
