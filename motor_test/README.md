# 鼓风机电机 CAN FD 自动化测试框架

## 环境要求

- Python 3.10+
- 硬件：YuntuCanLin Debugger（UART-to-CAN FD）
- 依赖安装：`pip install -r requirements.txt`

## 目录结构

```
motor_test/
├── can_interface.py         # CAN 通信抽象层
├── motor_client.py          # 电机控制高级 API
├── waveform_capture.py      # 波形采集与分析
├── anomaly_analyzer.py      # 异常检测与根因分析
├── auto_tuner.py            # 速度环/观测器自动调参
├── report_generator.py      # 测试报告生成
├── conftest.py              # pytest 共享 fixtures
├── test_cases/              # 测试用例
├── test_reports/            # 测试报告输出
└── test_records/            # 原始数据记录
```

## 运行测试

```bash
# 全部自动测试
python3 -m pytest motor_test/test_cases/ -v --port /dev/cu.usbserial-XXXX

# 单个测试
python3 -m pytest motor_test/test_cases/test_tc001_start_stop.py -v

# 生成报告
python3 motor_test/report_generator.py --input motor_test/test_records/ --output motor_test/test_reports/

# 自动调参
python3 motor_test/auto_tuner.py --port /dev/cu.usbserial-XXXX --mode speed_pi
```
