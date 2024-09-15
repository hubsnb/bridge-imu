//
// Created by Administrator on 2024/7/14.
//

#ifndef LQR_HPP
#define LQR_HPP

#include  "Eigen/Dense"
class StataSpace{
public:
        uint8_t row;
        uint8_t column;
        uint8_t num;
        Eigen::MatrixXf statespace;

        StataSpace(uint8_t row_init,uint8_t col_init) {
                row = row_init;
                column = col_init;
                statespace.resize(row,column);
        }

        void Assign(const float *A[]) {

        }



};

#endif //LQR_HPP
