/**
 * @file moving-average.hpp
 * @author Kevin Javier Gomez Guzman @kevgom018
 * @brief Provides a simple MovingAverage class to calculate the moving average over a fixed period.
 * @version 1.0
 * @date 2025-06-29
 * 
 * This class maintains a sliding window of the most recent values and computes their average.
 * If the number of values is less than the specified period, the average is not reported (returns -1.0).
 */

#include <queue>

namespace aon {
    
/**
 * @class MovingAverage
 * @brief Computes the moving average of the most recent values over a specified period.
 *
 * Example usage:
 * @code
 * aon::MovingAverage ma(5);
 * double avg = ma.update(10.0);
 * @endcode
 */
class MovingAverage {
  public:
      /**
       * @brief Constructs a MovingAverage object with a specified period.
       * @param period The number of recent values to consider for the moving average.
       */
      MovingAverage(int period) : period(period), sum(0.0) {}
  
      /**
       * @brief Updates the moving average with a new value.
       * 
       * Adds the new value to the window and updates the sum.
       * If the window has reached the specified period, returns the current moving average.
       * Otherwise, returns -1.0 to indicate insufficient data.
       * 
       * @param new_value The new data point to include in the moving average.
       * @return The current moving average if the window is full; otherwise, -1.0.
       */
      double update(double newValue) {
          window.push(newValue);
          sum += newValue;
  
          if (window.size() > period) {
              sum -= window.front();
              window.pop();
          }
  
          return window.size() == period ? sum / period : -1.0; // -1.0 means not enough data yet
      }
  
  private:
      int period;
      double sum;
      std::queue<double> window;
};

}