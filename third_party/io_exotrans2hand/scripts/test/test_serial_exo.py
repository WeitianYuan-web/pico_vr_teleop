import zenoh
import time
import sys
import struct
from io_bus_proto.io_bus_codec import encode_float64_multi_array,decode_float64_multi_array,decode_imu,proto_to_dict


zenoh_cf=zenoh.Config.from_file('./configs/config/zenoh.json5')

# 获取key list
def keys_list():
    t0=time.time()
    keys=[]
    with zenoh.open(zenoh_cf) as session:
        # 订阅所有key,打印key
        sub = session.declare_subscriber('**')
        for sample in sub:
            if sample.key_expr not in keys:
                keys.append(str(sample.key_expr))
            if time.time()-t0>1:
                break
    
    print(f'keys: {keys}')
    return keys

# 判断类型
def _msg_type(key):
    k=str(key)
    print(k)
    if 'tf' in k:
        return 'TFMessage'
    if 'joint' in k:
        return 'JointState'
    if 'vibration_feedback' in k:
        return 'Float64MultiArray'
    if 'joystick' in k:
        return 'Joy'
    if 'imu' in k:
        return 'Imu'
    if 'pose' in k:
        return 'PoseArray'
    return None


# 订阅key输出数据
def subscribe_key(key,s=10):
    t0=time.time()
    msg_type=_msg_type(key)
    # print(msg_type)
    with zenoh.open(zenoh_cf) as session:
        sub = session.declare_subscriber(key)
        for sample in sub:
            print(proto_to_dict(msg_type,sample.payload.to_bytes()))
            # if time.time()-t0>s:
            break


# 订阅key输出hz
def subscribe_key_hz(key):
    t0=time.time()
    count=0
    with zenoh.open(zenoh_cf) as session:
        sub = session.declare_subscriber(key)
        for sample in sub:
            count+=1
            # print(f'hz: {1/(time.time()-t0)}')
            if time.time()-t0>=1:
                print(f'{key}: hz: {count}hz')
                count=0
                t0=time.time()


def publish_key(key,data):
    try:
        # 转格式
        key='io_esk/vibration_feedback'
        data=encode_float64_multi_array(data)
        with zenoh.open(zenoh_cf) as session:
            pub = session.declare_publisher(key)
            while True:
                pub.put(data)
                time.sleep(0.03)
                    
                print(f'publish {key} {decode_float64_multi_array(data)}')
    except Exception as e:
        print(f'publish {key} error: {e}')



if __name__ == '__main__':
    try:
        print('keys_list:')
        x=keys_list()
        print('--------------------------------')
        for k in x:
            subscribe_key(k)
            time.sleep(0.1)
            print('--------------------------------')



        # 默认只有振动
        # publish_key('',[5,10,10,10,10,10,10,10,10,10])
        # subscribe_key_hz('io_esk/joint_data')
        # subscribe_key('io_esk/joint_data')
        # subscribe_key('io_esk/joystick_data',30)
        # subscribe_key('io_esk/vibration_feedback')
        
        # publish_key('io_esk/vibration_feedback',[10,10,10,10,10,10,10,10,10,10])
        
    except Exception as e:
        print(f'error: {e}')
    except KeyboardInterrupt:
        print('KeyboardInterrupt')
    finally:
        sys.exit(0)
