//
// Created by itr-wh on 4/18/24.
//

#ifndef AEMC_CLIENT_POINTSET_MATCHING_H
#define AEMC_CLIENT_POINTSET_MATCHING_H

#include "Eigen/Dense"
#include "vector"

static const int Length = 7;
/*
 * 创建通过读取文件或实时获取元数据的类别
 */


/*
 * 根据码字生成三维标准模板
 */
class StandardTemplate {
public:
    StandardTemplate(float radius, const std::vector<int> &codewords);

    ~StandardTemplate();

    std::vector<Eigen::MatrixX3f> GetPositionTemplate();

    void GenPositionTemplate();

private:
    float radius_;
    std::vector<int> codewords_;
    std::vector<Eigen::MatrixX3f> Position_templates;
    std::vector<std::vector<float>> Distance_templates;
};


/*
 * Pointset matching algorithms
 * Kernel correlation
 */

class PointsetMatching {
public:
    PointsetMatching();

    ~PointsetMatching();

/*
 * Kernel correlation algorithm
 * Input: The first point set, the second point set
 * Output: The matching result
 */
    int KC_algorithm(const std::vector<Eigen::MatrixX3f> &templates, const Eigen::MatrixX3f &pointset, float sigma);

private:

    // Kernel correlation between two point sets
    float KernelCorrelation(Eigen::MatrixX3f &pointset1, Eigen::MatrixX3f &pointset2, float sigma);

};


/*
 * 存在缺失的情况下,使用与上一帧的最近邻匹配
 */

#endif //AEMC_CLIENT_POINTSET_MATCHING_H
