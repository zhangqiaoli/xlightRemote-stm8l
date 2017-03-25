#ifndef __XLI_NODECONFIG_H
#define __XLI_NODECONFIG_H

// Node config protocol
/// MySensor Cmd = C_INTERNAL
/// MySensor Type = I_CONFIG

/// Use MySensor Sensor as Node Config Field (NCF), and payload as config data
#define NCF_MAP_SENSOR                  1       // Sensor Bitmap, payload length = 2
#define NCF_MAP_FUNC                    2       // Function Bitmap, payload length = 2
#define NCF_DEV_ASSOCIATE               10      // Associate node to device(s), payload length = 2 to 8, a device per uint16_t
#define NCF_DATA_ALS_RANGE              50      // Lower and upper threshholds of ALS, payload length = 2
#define NCF_DATA_TEMP_RANGE             51      // Tempreture threshholds, payload length = 2
#define NCF_DATA_HUM_RANGE              52      // Humidity threshholds, payload length = 2
#define NCF_DATA_PM25_RANGE             53      // PM2.5 threshholds, payload length = 2

/// Node function bit
#define NODE_FUNC_PIR_CONTROL           0       // Use PIR to control device on / off
#define NODE_FUNC_ALS_CONTROL           1       // Use ALS threshold to control device on / off
#define NODE_FUNC_ALS_CONSTANT          2       // Constant brightness level
#define NODE_FUNC_TEMP_CONSTANT         3       // Constant tempreture
#define NODE_FUNC_HUM_CONSTANT          4       // Constant humidity
#define NODE_FUNC_PM25_CONSTANT         5       // Constant air quality


#endif /* __XLI_NODECONFIG_H */
