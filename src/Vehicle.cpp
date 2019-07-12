/*
 * Vehicle.cpp
 *
 *  Created on: 12 Jun 2019
 *      Author: julian
 */

#include "Vehicle.h"

Vehicle::Vehicle() {}

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double t) {
	this->id = id;
	this->x = x;
	this->y = y;
	this->vx = vx;
	this->vy=vy;
	this->s=s;
	this->t=t;
}

void Vehicle::updateVehicle(double x, double y, double vx, double vy, double s, double t) {
	this->x = x;
	this->y = y;
	this->vx = vx;
	this->vy=vy;
	this->s=s;
	this->t=t;
}

Vehicle::~Vehicle() {}
