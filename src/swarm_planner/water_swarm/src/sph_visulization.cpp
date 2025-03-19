#include "sph_visulization.h"

std::vector<float> calculate_color(double esdf_value, double max_dist, double min_dist,
                                     std::vector<int> R_values, std::vector<int> G_values, std::vector<int> B_values)
{
    std::vector<float> color_result;

    /* 分段上色 */
    int colors_num = R_values.size();
    double dist_seg = (max_dist - min_dist) / colors_num;
    if (esdf_value > max_dist) esdf_value = max_dist;
    if (esdf_value < min_dist) esdf_value = min_dist;
    int seg_num = floor( (esdf_value - min_dist) / dist_seg );
    color_result.push_back((float)R_values[seg_num]/255.0);
    color_result.push_back((float)G_values[seg_num]/255.0);
    color_result.push_back((float)B_values[seg_num]/255.0);

    return color_result;
}

