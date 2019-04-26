# Force Torque Sensor Client

Ros node to capture `WrenchStamped` datapoints from a ros topic.

The class SensorClient offers the capture functionality. 
It needs 
- a `nodeHandle`, 
- a `wrenchTopic` string identifying the topic publishing the WrenchStamped-data, 
- `idleRate` double in Hz, which defines the timeout while waiting for a ros callback,
- `bufferSize`, defining how many wrench datapoints should be stored and used to nullify the data.

When starting the node wrenchTopic, idleRate and bufferSize are taken from the parameter server

As a ros node, the following services are possible
- `beginDataCollection`: 
    Begins the data collection (idle->reading)
    Parameters:
    - `float64 frequency`: 
        Frequency in Hz of the capturing process.
        If > 0, reads maximum of one datapoint every timeslot defined by frequency and discards further datapoints in the same timeslot.
        If < 0, passes all datapoints through.  
    - `string referenceFrame`: 
        The target reference frame. Both this frame and the frame in the header of the incoming WrenchStamped-datapoints must exist. 
    - `duration timeout`: 
        The maximum time duration the node should capture data. Captures indefinitely when duration is 0.  
    - `int32 maxWrenchCount`: 
        The maximum amount of wrenches to be captured. Captures indefinitely when set to 0.  
    - `bool nullifyData`: 
        When true, uses the average of the buffered datapoints to update the bias of the datapoints captured.
        **Even if this is false, the current stored bias is applied to the datapoints. Call nullify with `sampleSize=0` to reset the  bias.**
        Setting this to false can be helpful when doing multiple consecutive readings without wanting to reset the bias.

- `getCapturedData`: 
    Ends data collection, returns the accumulated amount of data points as an array of `WrenchStamped` points (reading->idle)
- `abortReading`: 
    Aborts the current reading, if is reading. (reading->idle)
- `nullify`: 
    Explicitly uses the average of the `sampleSize` last captured WrenchStamped datapoints, where `sampleSize` is a . Throws an error if there has not been 
    enough points to fill the internal buffer.
    Resets the bias to 0 if `sampleSize` equals 0.

