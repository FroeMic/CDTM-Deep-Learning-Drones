##### Suggested clean drone startup sequence #####
import time, sys
import ps_drone                                    # Import PS-Drone-API

if __name__ == '__main__':
    drone = ps_drone.Drone()                           # Start using drone
    drone.startup()                                    # Connects to drone and starts subprocesses

    drone.reset()                                      # Sets drone's status to good
    while (drone.getBattery()[0]==-1): time.sleep(0.1) # Wait until drone has done its reset
    print "Battery: "+str(drone.getBattery()[0])+"% "+str(drone.getBattery()[1]) # Battery-status
    drone.useDemoMode(False)                        # Give me everything...fast
    drone.setConfigAllID()
    drone.getNDpackage(["demo","pressure_raw","altitude","magneto","wifi","raw_measures","gyros_offsets",]) # Packets to decoded
    time.sleep(0.5)                                    # Give it some time to awake fully after reset

    ##### Mainprogram begin #####
    NDC = drone.NavDataCount
    end = False
    while not end:
        while drone.NavDataCount==NDC:  time.sleep(0.001) # Wait until next time-unit
        if drone.getKey():              end = True        # Stop if any key is pressed
        NDC=drone.NavDataCount
        print "-----------"
        print "Aptitude [X,Y,Z] :            "+str(drone.NavData["demo"][2])
        print "Altitude / sensor / pressure: "+str(drone.NavData["altitude"][3])+" / "\
              +str(drone.State[21])+" / "+str(drone.NavData["pressure_raw"][0])
        print "Megnetometer [X,Y,Z]:         "+str(drone.NavData["magneto"][0])
        print "Wifi link quality:            "+str(drone.NavData["wifi"])
        print "RawMeasures [X,Y,Z]:          "+str(drone.NavData["raw_measures"][0])
        print "Gyros Offset [X,Y,Z]:         "+str(drone.NavData["gyros_offsets"])
