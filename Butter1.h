#ifndef BUTTER1_HEADER
#define BUTTER1_HEADER

#include "Filter.h"

/*
 * First order butterworth lowpass filter
 */

class Butter1 : public Filter {
public:
  Butter1(double frequency, double cutoff);
  virtual void reset() override;
  virtual double process(double sample) override;
private:
  double xd1, yd1; // x[n - 1] and y[n - 1]
  double cx, cy; //coefficients for x[n], x[n - 1] (cx) and y[n-1] (cy)
};

#endif
