#ifndef VE_DIRECT_DCDC_H
#define VE_DIRECT_DCDC_H


class ViDirectDcDc {
  public:
  ViDirectDcDc();
  void parse(unsigned char buffer[],int p);
  void print();
  float in_v;
  float in_a;
  float in_w;

  float out_v;
  float out_a;
  float out_w;
  int cs;
  int zor;
  int er;
};

#endif