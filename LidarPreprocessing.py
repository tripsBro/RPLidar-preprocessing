from rplidar import RPLidar
# import pickle
# import simplejson
import math
import socket
import time
import json



# lidar = RPLidar('/dev/ttyUSB0')

def get_distance(flat1,flon1):
    global finalDestination
    x2lon=finalDestination[1]
    x2lat=finalDestination[0]
    print('get distance from current coordinate')
    diflat = math.radians(x2lat - flat1) #notice it must be done in radians
    flat1 = math.radians(flat1) #convert current latitude to radians
    x2lat = math.radians(x2lat) #convert waypoint latitude to radians
    diflon = math.radians((x2lon) - (flon1))  #subtract and convert longitudes to radians
    dist_calc = (math.sin(diflat / 2.0) * math.
                 sin(diflat / 2.0))
    dist_calc2 = math.cos(flat1)
    dist_calc2 *= math.cos(x2lat)
    dist_calc2 *= math.sin(diflon / 2.0)
    dist_calc2 *= math.sin(diflon / 2.0)
    dist_calc += dist_calc2
    dist_calc = (2 * math.atan2(math.sqrt(dist_calc), math.sqrt(1.0 - dist_calc)))
    dist_calc *= 6371000.0 #Converting to meters

    return dist_calc
def findPatch(dataList,number):
    patchList = []
    skip = 0
    for d,i in enumerate(dataList):
        if (skip > 0):  # to skip the values which have already been searched in recursive search.
            # print "skiping %d values" % skip
            skip -=1
            continue
        else:

            if i==number:
                patch = search(dataList,d)
                patchList.append(patch)
                skip = abs(patch[1] - patch[0])
                continue
            else:
                continue
    return patchList


def search(dataList,i):
    for j in range(i,len(dataList)-1):
        if dataList[j]== dataList[j+1]:
            continue
        else:
            stopIndex = j
            break

    else:
        stopIndex = len(dataList)
    return (i,stopIndex)



def lidar_process(scans,number,angle):
    lidarList = []

    requiredLidarList = [int(number)]*int(angle)

    for i in scans:
        lidarList.append((i[1],i[2])) # i[1] is the angle and i[2] is the distance
    for i in range(len(lidarList)):
        requiredLidarList.insert(int(lidarList[i][0]),lidarList[i][1])
    # print(requiredLidarList)
    patchList = findPatch(requiredLidarList,6000)
    if patchList is not None:
        for patch in patchList:
            if abs(patch[1]-patch[0])>0 and abs(patch[1]-patch[0])<=5: #because 5 degrees free is nothing for rover and it may be missing values by lidar
                startIndex = patch[0]
                if startIndex==0:
                    startIndex = 1
                for i in range(len(requiredLidarList)):
                    if (i== startIndex and i!=0):
                        for j in range(abs(patch[1]-patch[0])):
                            requiredLidarList[i+j] = requiredLidarList[i+j-1] + 1
                        break

                    elif (startIndex==0):
                        print("this was not supposed to happen!")
                    else:
                        continue
                continue
            elif (abs(patch[1]-patch[0])==0 and patch[0]!=0 and (patch[0]+ 1)<=(len(requiredLidarList)-2)):
                requiredLidarList[patch[0]] = (requiredLidarList[patch[0]-1] + requiredLidarList[patch[0]+ 1])/2
            else:
                pass
    return requiredLidarList


def play():
    jj = 0
    try:
        ssc = lidar.iter_scans(max_buf_meas=1000, min_len=5)
    except:
        print("Restarting")
        return play()

    while True:
        jj += 1
        try:
            state2 = next(ssc)
        except:
            ssc = lidar.iter_scans(max_buf_meas=1000, min_len=5)
            print("lidar still in configuration mode...")
            continue

        state = lidar_process(state2,6000,180)

        # state3 = json.dumps(state)
        # f = open('lidarValues.txt','w')
        # simplejson.dump(state,f)
        # f.write("\n")
        # f.close()
        # print("length: ", len(state))
        # print [(i,state[i]) for i in range(len(state))]
        print (state)

if __name__ == "__main__":


    # play()



    # to test the function, I have stored one of my readings in lidarValues.txt which i am using here.
    scan = [6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 2010.0, 6000, 2036.0, 2010.5, 1998.0, 1986.0, 6000, 1985.5, 1985.5, 1997.25, 2009.75, 2035.25, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 1985.25, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 294.0, 300.0, 6000, 284.0, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 1366.25, 1178.0, 1190.25, 6000, 301.25, 1187.0, 1190.25, 1198.25, 309.5, 1210.75, 6000, 1219.25, 1225.5, 1230.5, 1245.0, 6000, 1258.75, 1266.5, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 243.5, 6000, 237.0, 6000, 6000, 6000, 653.5, 662.0, 6000, 662.5, 659.5, 655.25, 655.75, 6000, 648.0, 6000, 6000, 6000, 429.75, 6000, 433.5, 424.5, 6000, 6000, 6000, 436.25, 447.0, 6000, 6000, 6000, 6000, 1440.25, 1422.75, 1440.75, 6000, 1460.25, 6000, 1467.0, 1467.0, 6000, 6000, 6000, 2455.0, 6000, 2455.0, 2473.0, 2474.25, 2512.75, 2514.5, 6000, 2553.0, 6000, 6000, 503.0, 6000, 489.0, 495.25, 6000, 6000, 6000, 6000, 6000, 362.75, 6000, 352.0, 6000, 6000, 6000, 6000, 350.5, 6000, 6000, 333.5, 331.25, 328.75, 6000, 325.25, 326.25, 328.75, 333.75, 341.75, 352.25, 360.75, 311.75, 302.0, 295.75, 291.5, 289.25, 287.75, 287.25, 287.75, 289.75, 291.25, 296.5, 301.25, 308.5, 315.5, 1961.25, 1949.5, 1940.5, 1938.5, 1938.0, 1937.0, 2187.5, 2144.25, 2101.5, 1825.25, 2010.0]
    scan = [(i,i,scan[i]) for i in range(len(scan))]
    state = lidar_process(scan,6000,180)
    print state


