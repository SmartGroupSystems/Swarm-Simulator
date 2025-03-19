#ifndef  _FORMATION_GRAD_H
#define  _FORMATION_GRAD_H

//Eigen
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

//ROS
#include <ros/ros.h>

//STANDARD
#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>
#include <numeric>
#include <string>
#include <memory>
#include <random>

//自定义
#include "EdtTransform.h"

using namespace std;


class FormationGrad
{
    public:
        int map_x_,map_y_;// vel_grid_ size
        double x_,y_;// vel_grid_ start point, start from the upper left corner
        double map_resolution_;
        double map_origin_x,map_origin_y;
        Eigen::MatrixXd FormationMatrix;
        Eigen::MatrixXd FormationGrad_;


    public:  
        FormationGrad(){}
        FormationGrad(const int &rows, const int &cols, const double &x,const double &y);
        ~FormationGrad(){}

        void setGrid();
        void setFormationGrad();
        double calcVelDistance(const Eigen::Vector3d& v);
        Eigen::Vector2d calcGrad(const Eigen::Vector3d& v);
        void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector2d pts[2][2], 
                            Eigen::Vector2d& diff);
        void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
        void interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, 
                            Eigen::Vector2d& grad);
        void interpolateBilinearDist(double values[2][2], const Eigen::Vector2d& diff,  
                            double& dist);       
        template<typename T>  inline T lerp(const T &lo, const T &hi, float t)  
        { return (lo * (0.1 - t) + hi * t)*10; }
        template <typename T> std::vector<size_t> sort_indexes(std::vector<T> &v)//matlab mink
        {   
            std::vector<size_t> idx(v.size());
            iota(idx.begin(), idx.end(), 0);
            sort(idx.begin(), idx.end(),
            [&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });
            return idx;// return index
        }

        inline Eigen::Vector2i posToIndex(const Eigen::Vector2d &pos)
        {
            Eigen::Vector2i curr_index;
            double dist_x = pos(0) - x_;
            double dist_y = y_ - pos(1);
            curr_index<< floor(dist_y/map_resolution_),floor(dist_x/map_resolution_);
            return curr_index;
        }

};

#endif