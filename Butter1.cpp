#include "Butter1.h"
#include <math.h>       /* tan */
#define PI 3.14159265
Butter1::Butter1(double frequency, double cutoff) : Filter(frequency, cutoff) {
  reset();  
}

void Butter1::reset() {
    xd1 = 0;
    yd1 = 0;

    double A = 1 / tan(cutoff * PI / frequency);
    cx = 1 / (A + 1);
    cy = (A - 1) / (A + 1);
}

double Butter1::process(double sample) {
  double out;

  out = cx * (sample + xd1) + cy * yd1;
  xd1 = sample; // x[n - 1] = x[n]
  yd1 = out; // y[n - 1] = y[n]

  return out;
}
