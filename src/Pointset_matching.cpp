//
// Created by itr-wh on 4/18/24.
//

#include "aemc_client/Pointset_matching.h"
#include <bitset>
#include "iostream"

StandardTemplate::StandardTemplate(float radius, const std::vector<int> &codewords) {
    // Constructor
    radius_ = radius;
    codewords_ = codewords;
}

StandardTemplate::~StandardTemplate() = default;

void StandardTemplate::GenPositionTemplate() {
    for (auto c: codewords_) {
        //按位读取码字的二进制
        std::bitset<Length> code(c);
        auto code_num = code.count();
        Eigen::MatrixX3f points(code_num, 3);
        int index = 0;
        for (int i = 0; i < Length; i++) {
            if (code[i] == 1 && index < code_num) {
                float x = radius_ * cos(2 * M_PI / Length * (Length - i));
                float y = radius_ * sin(2 * M_PI / Length * (Length - i));
                points.row(index) << x, y, 0;
                index++;
            }
        }
        Position_templates.push_back(points);
    }
}

std::vector<Eigen::MatrixX3f> StandardTemplate::GetPositionTemplate() {
    return Position_templates;
}


PointsetMatching::PointsetMatching() {
    // Constructor
}


PointsetMatching::~PointsetMatching() {
    // Destructor
}

int PointsetMatching::KC_algorithm(const std::vector<Eigen::MatrixX3f> &templates, const Eigen::MatrixX3f &pointset,
                                   float sigma) {
    // Kernel correlation algorithm
    // Input: The first point set, the second point set
    // Output: The matching result
    int best_match = -1;
    float max_correlation = -1;
    std::cout << std::fixed;
    std::cout.precision(2);
    for (int i = 0; i < templates.size(); i++) {
        Eigen::RowVector3f center = pointset.colwise().mean();
        Eigen::MatrixX3f centered_point = pointset.rowwise() - center;
        float correlation = KernelCorrelation(const_cast<Eigen::MatrixX3f &>(centered_point),
                                              const_cast<Eigen::MatrixX3f &>(templates[i]), sigma);
        if (correlation > max_correlation) {
            max_correlation = correlation;
            best_match = i;
        }
    }

//    std::cout << max_correlation << std::endl;
    return best_match;
}

float PointsetMatching::KernelCorrelation(Eigen::MatrixX3f &pointset1, Eigen::MatrixX3f &pointset2, float sigma) {
    // Kernel correlation between two point sets
    float correlation = 0;
    for (int i = 0; i < pointset1.rows(); i++) {
        for (int j = 0; j < pointset2.rows(); j++) {
//            std::cout << (pointset1.row(i) - pointset2.row(j)).squaredNorm() << std::endl;
//            correlation += exp(-((pointset1.row(i) - pointset2.row(j)).squaredNorm()) / (2 * sigma * sigma));
            correlation += -((pointset1.row(i) - pointset2.row(j)).squaredNorm()) / (2 * sigma * sigma);
        }
    }
}

/*
 * 将原始模板绕法向量旋转theta角下的最佳匹配,后将匹配结果与原始点集进行一一对应
 */
