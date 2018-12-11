#ifndef TOOLS_H
#define TOOLS_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>

#define PI 3.14159265359

double normalizeAngle(double angle);

template <typename T>
void toCSV(std::string path, T output, int row) {
    std::ofstream data_file;
    data_file.open(path.c_str(), std::ios_base::out | std::ios_base::app);

    if(data_file.is_open() == false) {
      std::cout << "ERROR OPENING THE FILE" << std::endl;
      return;
    }

    for(int i = 0; i < row; i++) data_file << ",";
    data_file << output << std::endl;

    data_file.close();
}

#endif // TOOLS_H
