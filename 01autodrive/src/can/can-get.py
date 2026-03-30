import can  
import csv  
from datetime import datetime  
import time  

class CANLogger:  
    def __init__(self, channel='can1', interface='socketcan', output_file='can_data.csv'):  
        self.bus = can.interface.Bus(channel=channel, bustype=interface)  
        self.csv_file = open(output_file, 'w', newline='')  
        self.writer = csv.writer(self.csv_file)  
        self._write_header()  
        
        # 初始化字段说明文件  
        with open('can_fields.csv', 'w', newline='') as f:  
            f.write("CAN ID,Signal Name,Start Bit,Length,Factor,Offset,Unit\n")  

    def _write_header(self):  
        headers = [  
            'Timestamp', 'CAN ID', 'Extended ID',  
            'DLC', 'Data 0', 'Data 1', 'Data 2', 'Data 3',  
            'Data 4', 'Data 5', 'Data 6', 'Data 7'  
        ]  
        self.writer.writerow(headers)  

    def _process_message(self, msg):  
        row = [  
            datetime.fromtimestamp(msg.timestamp).isoformat(),  
            hex(msg.arbitration_id),  
            msg.is_extended_id,  
            msg.dlc,  
            *msg.data  
        ]  
        # 补全数据长度到8字节  
        row += [0] * (8 - len(msg.data))  
        return row[:12]  # 保证每行12列  

    def start_logging(self):  
        try:  
            while True:  
                msg = self.bus.recv(timeout=1)  
                if msg is not None:  
                    self.writer.writerow(self._process_message(msg))  
                    self.csv_file.flush()  # 实时写入  
        except KeyboardInterrupt:  
            print("\nLogging stopped by user")  
        finally:  
            self.csv_file.close()  
            self.bus.shutdown()  

if __name__ == "__main__":  
    logger = CANLogger(channel='can1')  
    print("Starting CAN logging... (Press Ctrl+C to stop)")  
    logger.start_logging()  