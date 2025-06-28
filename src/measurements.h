#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

enum Sensor_type
  {
    SENSOR_ADS1115_A,
    SENSOR_ADS1115_V,
    SENSOR_INA226,
  };


#define MLEN 8

class Meas_va {
  public:
    String name;
    float A;
    float V;
};

class Power_conf {
  public:
    String name;
    Sensor_type sensor_type;
    int i2c_addr;
    int channel;
    float v_mult,v_offset;
    float a_mult,a_offset;
};

class Measurements {
    Meas_va meas_va[MLEN]; 
};

class Power_congigs {
    Power_conf power_conf[MLEN]; 
};

//extern Power_congigs power_congigs
//extern Measurements measurements;

#endif