/*------------ REFERENCE FOR SOFTWARESERVO FUNCTIONS -----------------*/

 	uint8_t attach(int);     			
 // attach to a pin, sets pinMode, returns 0 on failure, won't
 // position the servo until a subsequent write() happens
 	void detach();

    void write(int);
 // specify the angle in degrees, 0 to 180
    uint8_t read();
    uint8_t attached();
    void setMinimumPulse(uint16_t);
 // pulse length for 0 degrees in microseconds, 540uS default
    void setMaximumPulse(uint16_t);
 // pulse length for 180 degrees in microseconds, 2400uS default
    static void refresh();    
 // must be called at least every 50ms or so to keep servo alive
 // you can call more often, it won't happen more than once every 20ms