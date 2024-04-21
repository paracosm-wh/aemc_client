//
// Created by itr-wh on 4/19/24.
//

#include "Aemc.h"
#include "aemc_client/Pointset_matching.h"
#include "fstream"
#include "iostream"


int main(int argc, char *argv[]) {
    std::string path = R"(/home/itr-wh/Work/AEMC_data/RAL-data/1-L-middle1-raw.csv)";
//    std::string path = R"(/home/itr-wh/Work/AEMC_data/ICRA_data/Cluster-2.csv)";
    std::ifstream csv_data(path, std::ios::in);
    std::string line;

    std::ofstream outfile;
    outfile.open("/home/itr-wh/Work/AEMC_data/RAL-data/result/L-middle1-raw-result.csv",
                 std::ios::out | std::ios::trunc);
    if (!csv_data.is_open()) {
        std::cout << "Error: opening file fail" << std::endl;
        std::exit(1);
    }

    std::istringstream sin;
    std::string s;

    std::vector<double> points;
    std::vector<PreClusters> PreList;
    std::vector<PreTrace> PreTraceList;
    Eigen::MatrixX3d preSinglePoints;
    vector<CodedCluster> preCodedClusterList;
    while (std::getline(csv_data, line)) {
        sin.clear();
        sin.str(line);
        points.clear();

        int col_num = 0;
        int64_t frame;
        while (std::getline(sin, s, ',')) //将字符串流sin中的字符读到field字符串中，以逗号为分隔符
        {
            if (!s.empty()) {
                if (col_num == 0)
                    frame = static_cast<int64_t>(std::stod(s));
                else if (col_num > 1) {
                    points.push_back(std::stod(s));
                }
            }
            col_num++;
        }
        if (!points.empty()) {
            Eigen::MatrixX3d rawData;
            int nPoints = static_cast<int>(points.size());
            rawData.resize(nPoints / 3, 3);
            for (int i = 0; i < nPoints; ++i) {
                rawData(i / 3, i % 3) = points[i];

            }

            RawMarker raw1(frame, rawData);

            if (raw1.frame_ > 0) {
                // 数据处理
                std::vector<std::vector<int>> PotentialCluster_indices;
                raw1.EuclideanCluster(PotentialCluster_indices);
                std::vector<CodedCluster> codedClusterList;
                // 模板匹配使用
                std::vector<Eigen::MatrixX3f> templates;
                AEMC_I I_clusters(CodedLENGTH, 5);
                StandardTemplate std_template(classArr[5], I_clusters.GetCodewords());
                templates.push_back(std_template.Get()[0]);
                std::vector<Eigen::MatrixX3f> pointsSets;
                for (auto p: PotentialCluster_indices) {
                    if (p.size() != 4)
                        std::cout << frame << "p_size:" << p.size() << "," << raw1.raw_num << std::endl;
                    Eigen::MatrixX3f pointset(p.size(), 3);
                    for (int i = 0; i < p.size(); ++i) {
                        pointset.row(i) = raw1.Raw_data_.row(p[i]).cast<float>();
                    }
                    pointsSets.push_back(pointset);
                }
                PointsetMatching matching_algorithm;
                for (const auto &pointset: pointsSets) {
                    int best_match = matching_algorithm.KC_algorithm(templates, pointset, 1.5);
//                    std::cout << best_match << std::endl;
                    if (best_match != -1) {
                        Eigen::Vector3f center;
                        center = CircleFitting(pointset);

                        if (best_match == 1) {
//                            std::cout << center[0] << ',' << center[1] << ',' << center[2] << std::endl;
                            outfile << frame << ',';
                            outfile << center[0] << ',';
                            outfile << center[1] << ',';
                            outfile << center[2] << ',';
                        }
                    } else {
                        outfile << frame << ',';
                        outfile << 0 << ',';
                        outfile << 0 << ',';
                        outfile << 0 << ',';
                    }
                }
                outfile << std::endl;

            }
        }
    }
    return 0;
}