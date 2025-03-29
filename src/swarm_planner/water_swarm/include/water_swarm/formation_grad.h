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
#include <cmath>
#include <functional>

//自定义
#include "EdtTransform.h"

using namespace std;


struct FormationShape
{
    Eigen::Vector2d reference_point; // 参考点
    int shape_type;                  // 新增: 0-圆形, 1-矩形, 2-直线
    std::vector<double> params;      // 存储形状参数 (圆形: 半径; 矩形: 宽,高; 直线: 起点x,y和终点x,y)

    std::vector<std::function<double(const Eigen::Vector2d&)>> global_objectives;

    FormationShape(const Eigen::Vector2d& ref_pt, int type, const std::vector<double>& shape_params)
        : reference_point(ref_pt), shape_type(type), params(shape_params) {}

    void addObjective(const std::function<double(const Eigen::Vector2d&)>& obj)
    {
        global_objectives.push_back(obj);
    }
};

class FormationGrad
{
    public:
        int map_x_,map_y_;// vel_grid_ size
        double x_,y_;// vel_grid_ start point, start from the upper left corner
        double map_resolution_;
        double map_origin_x,map_origin_y;
        Eigen::MatrixXd FormationMatrix;
        Eigen::MatrixXd FormationGrad_;

        std::vector<FormationShape> formation_shapes_;

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

        // 新增队形的方法
        void addFormationShape(const FormationShape& shape)
        {
            formation_shapes_.push_back(shape);
        }

        // 计算势能函数梯度
        Eigen::Vector2d calcPotentialGrad(const Eigen::Vector3d& pos)
        {
            Eigen::Vector2d total_grad(0, 0);
            double k = 1.0; // 根据论文定义，k为正系数，可调节

            for (const auto& shape : formation_shapes_)
            {
                Eigen::Vector2d delta_q = pos.head<2>() - shape.reference_point;
                
                for (const auto& obj : shape.global_objectives)
                {
                    double fG = obj(delta_q);

                    // 计算数值梯度 ∂fG/∂q_i
                    Eigen::Vector2d grad_fG;
                    const double eps = 1e-5;
                    for (int i = 0; i < 2; ++i)
                    {
                        Eigen::Vector2d delta = Eigen::Vector2d::Zero();
                        delta(i) = eps;
                        double val_forward = obj(delta_q + delta);
                        double val_backward = obj(delta_q - delta);
                        grad_fG(i) = (val_forward - val_backward) / (2 * eps);
                    }

                    // 根据论文公式(27)计算梯度力
                    if (fG > 0)
                    {
                        total_grad += k * fG * grad_fG;
                    }
                    // 如果 fG ≤ 0，则梯度为0，不用处理（无力作用）
                }
            }

            return total_grad;
        }

        
        // FormationGrad 类中的实现
        std::vector<Eigen::Vector2d> getPolygonBoundaryPoints(const FormationShape& shape, int num_points_per_boundary = 100)
        {
            std::vector<Eigen::Vector2d> boundary_points;

            return boundary_points;
        }

};

#endif