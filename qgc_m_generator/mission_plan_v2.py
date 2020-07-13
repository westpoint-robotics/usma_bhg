#!/usr/bin/env python2

'''
https://medium.com/future-vision/google-maps-in-python-part-2-393f96196eaf

dep:
sudo apt-get install numpy python3-pandas
'''

import os

def make_item1(cmd, jumpid, frame, p1, p2, p3, p4):
    tmpstring = '\r\n\t\t\t{\r\n\t\t\t\t"autoContinue": true,\r\n\t\t\t\t"command": ' + str(cmd) + ','
    tmpstring += '\r\n\t\t\t\t"doJumpId": ' + str(jumpid) + ',\r\n\t\t\t\t"frame": ' + str(frame) + ',\r\n\t\t\t\t"params": ['
    tmpstring += '\r\n\t\t\t\t\t' + str(p1) + ',\r\n\t\t\t\t\t' + str(p2) + ',\r\n\t\t\t\t\t' + str(p3) + ',\r\n\t\t\t\t\t' + str(p4) + ',\r\n\t\t\t\t\t0,'
    tmpstring += '\r\n\t\t\t\t\t0,\r\n\t\t\t\t\t0\r\n\t\t\t\t],\r\n\t\t\t\t"type": "SimpleItem"'
    tmpstring += '\r\n\t\t\t}'
    return tmpstring

def waypoint_item(cmd, jumpid, frame, hold, pt_lat, pt_lon, alt):
    tmpstring ='\r\n\t\t\t{\r\n\t\t\t\t"AMSLAltAboveTerrain": null,\r\n\t\t\t\t"Altitude": ' + str(alt) + ','
    tmpstring += '\r\n\t\t\t\t"AltitudeMode": 1,'
    tmpstring += '\r\n\t\t\t\t"autoContinue": true,\r\n\t\t\t\t"command": ' + str(cmd) + ','
    tmpstring += '\r\n\t\t\t\t"doJumpId": ' + str(jumpid) + ',\r\n\t\t\t\t"frame": ' + str(frame) + ',\r\n\t\t\t\t"params": ['
    tmpstring += '\r\n\t\t\t\t\t' + str(hold) + ',\r\n\t\t\t\t\t0,\r\n\t\t\t\t\t0,\r\n\t\t\t\t\t0,'
    tmpstring += '\r\n\t\t\t\t\t' + str(pt_lat) + ',\r\n\t\t\t\t\t' + str(pt_lon) + ',\r\n\t\t\t\t\t' + str(alt) 
    tmpstring += '\r\n\t\t\t\t],\r\n\t\t\t\t"type": "SimpleItem"'
    tmpstring += '\r\n\t\t\t}'
    return tmpstring

def landing_item(cmd, jumpid, frame, pt_lat, pt_lon, alt):
    tmpstring ='\r\n\t\t\t{\r\n\t\t\t\t"AMSLAltAboveTerrain": null,\r\n\t\t\t\t"Altitude": 0 ,'
    tmpstring += '\r\n\t\t\t\t"AltitudeMode": 1,'
    tmpstring += '\r\n\t\t\t\t"autoContinue": true,\r\n\t\t\t\t"command": ' + str(cmd) + ','
    tmpstring += '\r\n\t\t\t\t"doJumpId": ' + str(jumpid) + ',\r\n\t\t\t\t"frame": ' + str(frame) + ',\r\n\t\t\t\t"params": ['
    tmpstring += '\r\n\t\t\t\t\t0,\r\n\t\t\t\t\t0,\r\n\t\t\t\t\t0,\r\n\t\t\t\t\tnull,'
    tmpstring += '\r\n\t\t\t\t\t' + str(pt_lat) + ',\r\n\t\t\t\t\t' + str(pt_lon) + ',\r\n\t\t\t\t\t0' 
    tmpstring += '\r\n\t\t\t\t],\r\n\t\t\t\t"type": "SimpleItem"'
    tmpstring += '\r\n\t\t\t}'
    return tmpstring

def rtl_item(cmd,jumpid,frame):
    tmpstring = '\r\n\t\t\t{\r\n\t\t\t\t"autoContinue": true,\r\n\t\t\t\t"command": ' + str(cmd) + ','
    tmpstring += '\r\n\t\t\t\t"doJumpId": ' + str(jumpid) + ',\r\n\t\t\t\t"frame": ' + str(frame) + ',\r\n\t\t\t\t"params": ['
    tmpstring += '\r\n\t\t\t\t\t0,\r\n\t\t\t\t\t0,\r\n\t\t\t\t\t0,\r\n\t\t\t\t\t0,\r\n\t\t\t\t\t0,'
    tmpstring += '\r\n\t\t\t\t\t0,\r\n\t\t\t\t\t0\r\n\t\t\t\t],\r\n\t\t\t\t"type": "SimpleItem"'
    tmpstring += '\r\n\t\t\t}'
    return tmpstring

def make_plan(mission_input, speed):
    #print("Waypoints are: ", waypoints)
    outstring =  ''
    outstring += '{\r\n\t"fileType": "Plan",\r\n\t"geoFence": {\r\n\t\t"circles": [\r\n\t\t],'
    outstring += '\r\n\t\t"polygons": [\r\n\t\t],\r\n\t\t"version": 2\r\n\t},'
    outstring += '\r\n\t"groundStation": "QGroundControl",\r\n\t"mission": {'
    #  cruiseSpeed: is The default forward speed for Fixed wing or VTOL vehicles
    # firmwareType: The firmware type for which this mission was created (3 is MAV_AUTOPILOT_ARDUPILOTMEGA)
    #   hoverSpeed: The default forward speed for multi-rotor vehicles.
    outstring += '\r\n\t\t"cruiseSpeed": 15,\r\n\t\t"firmwareType": 3,\r\n\t\t"hoverSpeed": ' + str(speed) + ','
    outstring += '\r\n\t\t"items": ['
    # Writing Mission Plan
    for i in range(len(mission_input)):
        if len(mission_input[i]) <= 3:
            if mission_input[i][0] == 115:     # 115 is MAV_CMD_CONDITION_YAW
                if mission_input[i][1] < 0:
                    outstring += make_item1(115,i+1,0,-mission_input[i][1],5,-1,mission_input[i][2])
                else:
                    outstring += make_item1(115,i+1,0,mission_input[i][1],5,1,mission_input[i][2])
            elif mission_input[i][0] == 20:    #  20 is MAV_CMD_RTL
                outstring += rtl_item(20,i+1,2)
            elif mission_input[i][0] == 178:   #  178 is MAV_CMD_DO_CHANGE_SPEED
                outstring += make_item1(178,i+1,2,1,mission_input[i][1],-1,0)
        else:
            if mission_input[i][0] == 21:      #  21 is MAV_CMD_LANDING
                wp_lat,wp_lon,alt = mission_input[i][1], mission_input[i][2], mission_input[i][3]
                outstring += landing_item(21,i+1,3,wp_lat,wp_lon,alt)
            else:                              #  16 is MAV_CMD_NAV_WAYPOINT
                wp_lat,wp_lon,alt,hold = mission_input[i][0], mission_input[i][1], mission_input[i][2], mission_input[i][3]
                outstring += waypoint_item(16,i+1,3,hold,wp_lat,wp_lon,alt)
        if i != len(mission_input)-1:
            outstring += ','
    outstring += '\r\n\t\t],\r\n\t\t"plannedHomePosition": [\r\n\t\t\t41.3802304,'
    outstring += '\r\n\t\t\t-73.9723826,\r\n\t\t\t228\r\n\t\t],\r\n\t\t"vehicleType": 2,'
    outstring += '\r\n\t\t"version": 2\r\n\t},\r\n\t"rallyPoints": {\r\n\t\t"points": ['
    outstring += '\r\n\t\t],\r\n\t\t"version": 2\r\n\t},\r\n\t"version": 1\r\n}'
    
    # Replace tabs with 4 spaces. This matches the original file.
    outstring = outstring.replace('\t','    ')
    return outstring

if __name__ == '__main__':
    wpts = [[41.3906892,-73.9533453,10,3],
            [41.3915474,-73.9529322,10,2],
            [41.3915681,-73.9530079,10,3],
            [41.3915681,-73.9530079,10,5],
            [41.3907099,-73.9534210,10,0]            
           ]
    s = 2.5
    y = 289
    with open('test.txt', 'w') as f:
        f.write(make_plan(wpts,s,y))
