# Refer to `test/shopping_cart/setting/bluetooth.py`
from bleak import BleakScanner
import numpy as np
import asyncio

class Bluetooth:
    def __init__(
        self, 
        target_device = "90:E2:02:9F:D5:E5",
        calibration = 0,
    ):
        self.scanner = BleakScanner()
        self.calibration = calibration
        self.rssi_list = []
        self.distance_list = []
        self.mid = 0
        self.distance = 0
        self.target_device = target_device

    async def calibrate_run(self):
        def detection_callback(device, advertisement_data):
            if device.address == self.target_device:
                rssi = device.rssi
                if(len(self.rssi_list)<10):                         # first ten 
                    self.rssi_list.append(rssi)
                    self.mid = np.mean(self.rssi_list)

                elif(len(self.rssi_list)<=30):                            
                    if(rssi >= self.mid -4 and rssi <= self.mid +5):
                        self.rssi_list.append(rssi)
                        self.mid = np.mean(self.rssi_list)

                else:
                    if (self.calibration==0):
                        self.calibration = self.mid
                        print(f"\rcalibrated {self.calibration}")
                print(f"\r{len(self.rssi_list)}/30 RSSI: {rssi:.2f}, mid: {self.mid:.2f},",end="")
                loop = asyncio.get_event_loop()
                loop.create_task(self.scanner.stop())

        self.scanner.register_detection_callback(detection_callback)
        await self.scanner.start()
        await asyncio.sleep(0.2)
        await self.scanner.stop()

    async def calculate_dist(self):
        def detection_callback(device, advertisement_data):
            weight = [1,0.8,0.7,0.4,0.2,0.1]
            if device.address == self.target_device:
                rssi = device.rssi
                if (self.calibration!=0):
                    if(len(self.rssi_list)>6):
                        self.rssi_list = self.rssi_list[0:6]
                    
                    # if (len(self.rssi_list)<6):
                    #     self.rssi_list.append(rssi)

                    elif(len(self.rssi_list)==6):
                        self.rssi_list.pop()
                        self.rssi_list.insert(0,rssi)
                        final_distance = 10**((self.calibration-np.average(self.rssi_list, weights=weight))/(10*2.4))
                        print("final distance: ",final_distance)
                        self.distance = final_distance

                loop = asyncio.get_event_loop()
                loop.create_task(self.scanner.stop())

        self.scanner.register_detection_callback(detection_callback)
        await self.scanner.start()
        await asyncio.sleep(0.2)
        await self.scanner.stop()

    def initializing(self):
        while self.calibration == 0:
            loop = asyncio.get_event_loop()
            loop.run_until_complete(self.calibrate_run())
        while self.distance == 0:
            loop = asyncio.get_event_loop()
            loop.run_until_complete(self.calculate_dist())
        print("bluetooth initializing done")

    def calculate_distance(self):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.calculate_dist())
        return self.distance