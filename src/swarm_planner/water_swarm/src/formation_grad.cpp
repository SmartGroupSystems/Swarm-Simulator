#include "formation_grad.h"

FormationGrad::FormationGrad(const int &rows, const int &cols, const double &x,
        const double &y)
{
    map_x_ = rows;
    map_y_ = cols;
    x_ = x;
    y_ = y;
    map_resolution_ = 2*fabs(x_)/map_x_;
    map_origin_x = x_ + map_resolution_/2;
    map_origin_y = y_ - map_resolution_/2;
    FormationMatrix.setZero(map_x_,map_y_);
    FormationGrad_.resize(map_x_,map_y_);
    setGrid();
    setFormationGrad();
}

void FormationGrad::setGrid() 
{
    // FormationMatrix.setZero(map_x_, map_y_);
    FormationMatrix.setOnes(map_x_, map_y_);
    // 遍历所有栅格，计算其实际坐标
    for (size_t i = 0; i < map_x_; i++) {
        for (size_t j = 0; j < map_y_; j++) {
            // 计算当前栅格的世界坐标
            Eigen::Vector3d current_pt;
            current_pt.x() = map_origin_x + i * map_resolution_; // x 方向增加
            current_pt.y() = map_origin_y - j * map_resolution_; // y 方向减少
            current_pt.z() = 0.0;
            // 仅在 (0.5 ≤ x ≤ 7.5, 0.5 ≤ y ≤ 7.5) 区域内填充 1
            if (current_pt.x() >= -1.5 && current_pt.x() <= 8.5 &&
                current_pt.y() >= -1.5 && current_pt.y() <= 8.5) {
                // FormationMatrix(j, i) = 1;  // 填充 1
                FormationMatrix(j, i) = 0; 
            }
        }
    }
}

void FormationGrad::setFormationGrad()
{
    double *src_v,*dst_v, *src1_v, *dst1_v;
    const int map_size = FormationGrad_.rows() * FormationGrad_.cols();

    src_v  = (double*)malloc(map_size*sizeof(double));
    dst_v  = (double*)malloc(map_size*sizeof(double));
    src1_v = (double*)malloc(map_size*sizeof(double));
    dst1_v = (double*)malloc(map_size*sizeof(double));

    for (size_t i = 0; i < FormationGrad_.rows(); i++)
    {
    for (size_t j = 0; j < FormationGrad_.cols(); j++)
    {
        *(src_v  + i*FormationGrad_.rows()+ j) = FormationMatrix(i,j);
        *(src1_v + i*FormationGrad_.rows()+ j) = 1.0 - FormationMatrix(i,j);
    }
    }    

    computeEDT(dst_v , src_v , FormationGrad_.rows(), FormationGrad_.cols());
    computeEDT(dst1_v, src1_v, FormationGrad_.rows(), FormationGrad_.cols());

    for (size_t i = 0; i < FormationGrad_.rows(); i++)
    {
    for (size_t j = 0; j < FormationGrad_.cols(); j++)
    {
        if(FormationMatrix(i,j)==0)
        FormationGrad_(i,j) =  (int)sqrt(*(dst_v  + i*FormationGrad_.rows()+j));
        else
        FormationGrad_(i,j) = -(int)sqrt(*(dst1_v + i*FormationGrad_.rows()+j))+1;
    }
    }  

    free(src_v);    src_v  = NULL; 
    free(dst_v);    dst_v  = NULL;
    free(src1_v);   src1_v = NULL;
    free(dst1_v);   dst1_v = NULL;
}

double FormationGrad::calcVelDistance(const Eigen::Vector3d& v)
{
    double dist;
    Eigen::Vector2d diff;
    Eigen::Vector2d sur_pts[2][2];
    getSurroundPts(v,sur_pts, diff);
    double dists[2][2];
    getSurroundDistance(sur_pts, dists);
    interpolateBilinearDist(dists, diff, dist);
    return dist;
}

Eigen::Vector2d FormationGrad::calcGrad(const Eigen::Vector3d& v)
{
    Eigen::Vector2d dist_grad;
    Eigen::Vector2d diff;
    Eigen::Vector2d sur_pts[2][2];
    getSurroundPts(v,sur_pts, diff);
    double dists[2][2];
    getSurroundDistance(sur_pts, dists);
    interpolateBilinear(dists, diff, dist_grad);
    return dist_grad;
}

void FormationGrad::getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector2d pts[2][2], 
                                Eigen::Vector2d& diff)
{
    double dist_x = pos(0) - x_;
    double dist_y = y_ - pos(1);
    diff(0) = fmod(dist_x,map_resolution_);
    diff(1) = fmod(dist_y,map_resolution_);

    Eigen::Vector2d curr_index;
    curr_index<< floor(dist_y/map_resolution_),floor(dist_x/map_resolution_);
    for (size_t i = 0; i < 2; i++)
    {
    for (size_t j = 0; j < 2; j++)
    {       
        Eigen::Vector2d tmp_index(curr_index(0)+i,curr_index(1)+j);
        pts[i][j] = tmp_index;
    }
    }

}

void FormationGrad::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2])
{
    for (size_t i = 0; i < 2; i++)
    {
    for (size_t j = 0; j < 2; j++)
    {
        Eigen::Vector2d tmp_index = pts[i][j];
        dists[i][j] = FormationGrad_(tmp_index(0),tmp_index(1));   
    }
    }
}

void FormationGrad::interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, 
                                Eigen::Vector2d& grad)
{
    double c00 = values[0][0];
    double c01 = values[0][1];
    double c10 = values[1][0];
    double c11 = values[1][1];
    double tx = diff(0);
    double ty = diff(1);
    
    double nx0 = lerp(c00,c10,ty);
    double nx1 = lerp(c01,c11,ty);
    double ny0 = lerp(c00,c01,tx);
    double ny1 = lerp(c10,c11,tx);

    grad(0) = (nx1- nx0)/map_resolution_;
    grad(1) = (ny0- ny1)/map_resolution_;
}

void FormationGrad::interpolateBilinearDist(double values[2][2], const Eigen::Vector2d& diff,  
                    double& dist)
{
    double c00 = values[0][0];
    double c01 = values[0][1];
    double c10 = values[1][0];
    double c11 = values[1][1];
    double tx = diff(0);
    double ty = diff(1);
    
    double nx0 = lerp(c00,c10,ty);
    double nx1 = lerp(c01,c11,ty);
    double ny0 = lerp(c00,c01,tx);
    double ny1 = lerp(c10,c11,tx);

    dist = lerp(ny0,ny1,ty);
}
