# Refer to `test/shopping_cart/setting/bluetooth.py`
from bleak import BleakScanner, BleakClient
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

                elif(len(self.rssi_list)<30):                            
                    if(rssi >= self.mid -4 and rssi <= self.mid +5):
                        self.rssi_list.append(rssi)
                        self.mid = np.mean(self.rssi_list)

                else:
                    if (self.calibration==0):
                        self.calibration = self.mid
                        # print(f"\rcalibrated {self.calibration}")
                print(f"\r{len(self.rssi_list)}/30 RSSI: {rssi:.2f}, mid: {self.mid:.2f},",end="")
                loop = asyncio.get_event_loop()
                loop.create_task(self.scanner.stop())

        self.scanner.register_detection_callback(detection_callback)
        await self.scanner.start()
        await asyncio.sleep(0.2)
        await self.scanner.stop()

    async def calculate_dist(self):
        def detection_callback(device, advertisement_data):
            weight = [1.0,1.0,0.8,0.3,0.3]
            if device.address == self.target_device:
                rssi = device.rssi
                if (self.calibration!=0):
                    if(len(self.rssi_list)>5):
                        self.rssi_list = self.rssi_list[0:5]
                    
                    # if (len(self.rssi_list)<6):
                    #     self.rssi_list.append(rssi)

                    elif(len(self.rssi_list)==5):
                        
                        self.rssi_list.pop()
                        self.rssi_list.insert(0,rssi)
                        final_distance = 10**(((self.calibration-np.average(self.rssi_list, weights=weight))/(10*3)))/2
                        # print("final distance: ",final_distance)
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
    

class Bluetooth2:
    def __init__(
        self, 
        # target_device = "90:E2:02:9F:D5:E5",
        target_device = "9C:92:4F:4C:10:34",
        calibration = 0,
    ):
        self.scanner = BleakScanner()
        self.calibration = 0.7
        self.rssi_list = []
        self.distance_list = []
        self.mid = 0.7
        self.distance = 0
        self.target_device = target_device

        def detection_callback(device, advertisement_data):
            if device.address == self.target_device:
                rssi = device.rssi
                print(f"RSSI: {rssi}")
        self.scanner.register_detection_callback(detection_callback)

    async def _search(self):
        devices = await self.scanner.discover()
        for d in devices:
            if d.address == self.target_device:
                print(d.address)

    def search(self):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self._search())

    async def calculate_dist(self):
        await self.scanner.start()
        await asyncio.sleep(0.5)
        await self.scanner.stop()

    def calculate_distance(self):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.calculate_dist())
        return self.distance

    async def _connect(self):
        hw_address = "9C:92:4F:4C:10:34"

        def detection_callback(device, advertisement_data):
            if device.address == hw_address:
                rssi = device.rssi
                print(f"RSSI: {rssi}")

        async with BleakClient(hw_address) as client:
            # client.set_disconnected_callback(on_disconnect)
            print(f"connected!")
            print(f"connected!, {client}")
            
            # services = await client.get_services()
            # for service in services:
            #     print(service)

    def connect(self):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self._connect())

