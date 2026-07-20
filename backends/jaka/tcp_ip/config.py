"""JAKA TCP/IP 示例程序默认配置。"""

# 机械臂控制器 IP
ROBOT_IP = "192.168.10.11"

# 以下账号密码用于 JAKA SDK 登录（V3.2.10+），TCP/IP 协议本身不需要认证
ROBOT_USERNAME = "jakazuadmin"
ROBOT_PASSWORD = "jakazuadmin"

# 控制端口（文档固定）
CMD_PORT = 10001
STATUS_PORT = 10000
