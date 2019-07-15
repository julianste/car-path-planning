#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int id, double x, double y, double vx, double vy, double s, double d);

  // Destructor
  virtual ~Vehicle();

  void updateVehicle(double x, double y, double vx, double vy, double s, double d);


  int id;
  float x,y,vx,vy,s,d;
};

#endif  // VEHICLE_H
