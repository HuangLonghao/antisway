#ifndef FILTER_HEADER
#define FILTER_HEADER

/*
 * Base class for filters
 */

class Filter {
public:
  /*
   * cutoff - cutoff frequency of the filter
   * frequency - sampling frequency
   */
  double cutoff, frequency;

  Filter() : Filter(10, 100) {};
  Filter(double cutoff, double frequency) : cutoff(cutoff), frequency(frequency) { };
  /*
   * Zeros filter's memory and recalculates coefficients based on cutoff and sampling frequency
   */
  virtual void reset() = 0;
  /*
   * Passes a single sample through the filter
   * Returns the result of filtering
   */
  virtual double process(double sample) = 0;
};

#endif 
