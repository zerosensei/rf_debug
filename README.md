### rf2.4G转串口

- 发送端配置CONFIG_RF_MODE=1

- 接收端配置CONFIG_RF_MODE=2

- 发送端串口脚为串口0，默认波特率921600，串口接收到数据通过rf发送

- 接收端USB模拟串口，rf接收到数据通过USB上传

- 实测最高速度70KB/s，70KB/s在通信质量良好的情况下几乎不丢包

