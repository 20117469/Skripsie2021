import sqlite3
import paho.mqtt.client as mqtt

mqttClusterDataReceived = False
mqttClusterData = ""
mqttFireWarningReceived = False
mqttFireWarningData = ""

mqttBroker = "test.mosquitto.org"

#def main():
#    return



#if __name__ == '__main__':
#    main()


#                        MQTT functions
###################################################################
def publishToInstructionsTopic(data):
    client = mqtt.Client("wildfireDetectionServerInstructions")
    client.connect(mqttBroker)
    client.publish(f"wildfireDetection/Instructions", data)

def onClusterDataReceived(client, userdata, message):
    mqttClusterDataReceived = True
    mqttClusterData = str(message.payload.decode("utf-8"))

def onFireWarningReceived(client, userdata, message):
    mqttFireWarningReceived = True
    mqttFireWarningData = str(message.payload.decode("utf-8"))

def subscribeToClusterDataTopic():
    client = mqtt.Client("wildfireDetectionServerDataWatch")
    client.connect(mqttBroker)
    client.loop_start()
    client.subscribe(f"wildfireDetection/Cluster_Data")
    client.on_message() = onClusterDataReceived

def subscribeToFireWarningTopic():
    client = mqtt.Client("wildfireDetectionServerFireWatch")
    client.connect(mqttBroker)
    client.loop_start()
    client.subscribe(f"wildfireDetection/Fire_Warning")
    client.on_message() = onFireWarningReceived

###################################################################


#                       DB functions
################################################################
def dbInsertIntoClusterGatewayTable(ID, longt, lat, loraChannel):
    conn = sqlite3.connect('wildfiresqlite.db')
    gatewayID = ID
    longtitude = longt
    latitude = lat
    loraSamplingChannel = loraChannel
    sqlCommandString = f"INSERT INTO clusterGatewayTable (gatewayID,longtitude,latitude,loraSamplingChannel) \
                        VALUES ({gatewayID}, {longtitude}, {latitude}, {loraSamplingChannel} )"
    conn.execute(sqlCommandString)
    conn.close()

def dbInsertIntoMyDataTable(entryID, gateID, segmentNum, sensID, temp, relHumid, rainfall, wind, ,batlvl, timeUpdated):
    conn = sqlite3.connect('wildfiresqlite.db')
    ID = entryID
    gatewayID = gateID
    segmentNumber = segmentNum
    sensorID = sensID
    temperature = temp
    relativeHumidity = relHumid
    rainfallDaily = rainfall
    windSpeed = wind
    batlevel = batlvl
    lastUpdated = timeUpdated
    sqlCommandString = f"INSERT INTO myData (id, gatewayID,segmentNumber,sensorID,temperature, relativeHumidity, rainfallDaily, windSpeed, batLevel, lastUpdated) \
                        VALUES ({ID}, {gatewayID}, {segmentNumber}, {sensorID}, {temperature},{relativeHumidity}, {rainfallDaily}, {windSpeed}, {batlevel}, {lastUpdated})"
    conn.execute(sqlCommandString)
    conn.close()

def dbInsertIntofireAlertTable(entryID, gateID, segNumber, sensID, timeDet):
    conn = sqlite3.connect('wildfiresqlite.db')
    ID = entryID
    gatewayID = gateID
    segmentNumber = segNumber
    sensorID = sensID
    timeDetected = timeDet
    sqlCommandString = f"INSERT INTO fireAlert (id,gatewayID,segmentNumber,sensorID, timeDetected) \
                        VALUES ({ID}, {gatewayID}, {segmentNumber}, {sensorID},{timeDetected})"
    conn.execute(sqlCommandString)
    conn.close()
        

def dbGetLatestEntriesFromClusterID(clusterID):
    conn = sqlite3.connect('wildfiresqlite.db')
    sqlCommandString = f"SELECT id, gatewayID, segmentNumber, sensorID, temperature, relativeHumidity, rainfallDaily, windSpeed, batlevel, lastUpdated \
                        From myData \
                        WHERE gatewayID = {clusterID} \
                        ORDER BY lastUpdated DESC \
                        LIMIT 4"
    dbDATA = conn.execute(sqlCommandString)
    conn.close()
    return dbDATA

def dbGetClusterIdData(clusterID):
    conn = sqlite3.connect('wildfiresqlite.db')
    sqlCommandString = f"SELECT gatewayID, longtitude, latitude, loraSamplingChannel \
                        From clusterGatewayTable \
                        WHERE gatewayID = {clusterID}"

    dbDATA = conn.execute(sqlCommandString)
    conn.close()
    return dbDATA

def dbGetFireAlertData(numOfEntries):
    conn = sqlite3.connect('wildfiresqlite.db')
    sqlCommandString = f"SELECT id, gatewayID, segmentNumber, sensorID, timeDetected \
                        From fireAlert \
                        ORDER BY timeDetected DESC\
                        LIMIT {numOfEntries}"
    dbDATA = conn.execute(sqlCommandString)
    conn.close()
    return dbDATA

##################################################################

#              Calculated FDI Values for a Cluster ID
###################################################################
def calcNewFDIValuesForCLusterID(clusterID):
    data = dbGetLatestEntriesFromClusterID(clusterID)
    fdiArray = [[0 for i in range(4)] for j in range(4)]
    count = 0

    for row in data:
        temp = row[4]
        humid = row[5]
        rainfall = row[6]
        windSpeed = row[7]


        
###################################################################