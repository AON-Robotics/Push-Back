

#WARNING: INSTALL RPLIDAR LIBRARY BY ROBOTICIA BEFORE RUNNING

from rplidar import RPLidar

class RPlidarA1:
    #initialize connection
    def __init__(self, port):
        self.lidar= RPLidar(port)
    
    #Starts the lidar scanning 
    def Start_scanning(self):
        self.lidar.start_motor()
        self.curr_measures = self.lidar.iter_measurments()

    #returns the current scan information
    def get_scan(self):
        new_scan, quality, angle, distance =next(self.curr_measures)
        return angle, distance
    
    #Stops the sensor from scanning
    def Stop_scanning(self):
        self.lidar.stop() #stops scanning NOT motor 
        self.lidar.stop_motor() #stops motor 


    #disconnects the sensor 
    def disconnect(self):
        self.lidar.disconnect() 


    #Prints out and returns a dictionary with sensor information
    def get_device_info(self) -> dict:
        self.lidar.stop() #make sure we arent scanning anymore 
        self.lidar.clean_input() #remove potential old scan data
        info = self.lidar.get_info()
        print(info)
        return info
    

    #Prints out and returns device health state. 
    # When the system detects a risk that may cause hardware failure the returned status  will be 'Warning'. (sensor can still work)
    # When the sensor is in a Protection Stop state, the returned status will be 'Error'.
    #Other wise the returned status will be 'Good'
    def get_health(self):
        self.lidar.stop() #make sure we arent scanning anymore 
        self.lidar.clean_input() #remove potential old scan data
        health_stats= self.lidar.get_health()
        print(health_stats)
        return health_stats
    
    
    #Given a scan limit (the number of full rotations you want the sensor to do ) this function will return and print
    # what was scanned. 
    def get_Scan_Data(self, scan_limit=3):
        self.lidar.clean_input() #we dont want old data showing up 
        scans = 0  #1 scan = 360 degree rotation
        data= []

        
        for scan in self.lidar.iter_scans():
            scans+=1
            print(f"\n  SCAN # {scans}")

            for quality, angle, distance in scan:
                measurments = (scans, angle , distance , quality)
                data.append(measurments)
                print (f"/n Lidar angle (degrees):{ angle:.2f} , distance(mm): {distance}")
        
            if scans==scan_limit:
                break

        return data

