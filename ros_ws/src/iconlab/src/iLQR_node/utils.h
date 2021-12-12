/*
 * utils.h
 *
 *  Created on: Jan 26, 2021
 *      Author: talhakavuncu
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include "iLQR.h"
#include <fstream>

struct stateVector {
    float x, y, z;
    float roll, pitch, yaw;
    float x_d, y_d, z_d;
    float roll_d, pitch_d, yaw_d;
};

struct pose {
    float x, y, z;
    float roll, pitch, yaw;
    unsigned long timeStamp;
};

template<size_t st_sz,size_t ac_sz,size_t horizon>
void write_file(std::string& state_path,std::string& input_path,
		typename iLQR<st_sz,ac_sz,horizon>::state_input_trajectory soln);

void init_datalog(std::ofstream& file, std::string path, std::string name);

void logDroneStates(std::ofstream& file, std::vector<stateVector> states);

// void logOptimizer(std::ofstream& file, std::vector<)

#include "utils.cpp"

#endif /* UTILS_H_ */
