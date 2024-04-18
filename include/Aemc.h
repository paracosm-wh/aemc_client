//
// Created by itr-wh on 22-12-29.
//

#ifndef AEMC_CLIENT_AEMC_H
#define AEMC_CLIENT_AEMC_H

# include "Eigen/Dense"

# include "vector"
# include "list"
# include "map"

using namespace Eigen;
using namespace std;

//const vector<float> classArr{17.29, 42.0, 50.0, 28.0, 39.0, 43.0};
const vector<float> classArr{17.29, 0.0, 0.0, 35, 39.0};
const float stand_deviation = 2.5;
const float ERR = 2 * stand_deviation;
const int MinClusterNum = 3;
const int CodedLENGTH = 7;

struct Circle {
    float radius;
    Eigen::Vector3d center;
};

struct CodedCluster {
    Eigen::MatrixX3d Points_;   // Ordered points
    bool Coded_flag = false;
    int Codeword_;           // Two types of codewords: int and string
    string id;
    float Radius;
    Eigen::Vector3d Position;
    Eigen::Quaternion<double> Pose;
    float Confidence;
    //debug
    vector<float> chords;
};

struct PreClusters {
    string id;
    vector<int64_t> frames;
    vector<Eigen::Vector3d> Positions;
};

struct PreTrace {
    vector<string> ids;
    vector<int64_t> frames;
    vector<Eigen::Vector3d> Positions;
};

//将每个点赋予位置，无帧头赋予顺序，有帧头赋予位置
//Explicit and Implicit headers

class AEMC_E {
/* Explicit clusters
 * Properties:
 * * CodeLength
 * * Radius
 * * Codewords
 * * Chord_matrix
 * Functions:
 * * GenCodewords
 * * GenChordMatrix
 *
 */
private:
    int CodeLength_;
    float Radius_;
    vector<int> Codewords;
    Eigen::Matrix<float, 7, 7> Chord_matrix;

    void GenCodewords();

    void GenChordMatrix();

public:
    AEMC_E(int CodeLength, int classBit);

    Eigen::MatrixXf GetChordMatrix();

    vector<int> GetCodewords();

};

/* Implicit clusters
 * Properties:
 * * CodeLength
 * * Radius
 * * Codewords
 * *
 * Functions:
 * * GenCodewords
 * * GetRadius
 * * Decoding
 */
class AEMC_I {
private:
    int CodeLength_;   // Length of codewords
    float Radius_;
    vector<int> Codewords;
    vector<float> Base_chords;

    void GenCodewords();

    void GenBase();

public:
    AEMC_I(int CodeLength, int classIndices);

    vector<float> &GetBaseChords();

    vector<int> GetCodewords();
};


class RawMarker {
private:

//    int code_num; //CodedMarker数量
    Eigen::VectorXf Uncoded_marker;
    vector<CodedCluster> Coded_marker;
public:
    int64_t frame_;
    int raw_num; //原始点数量
    Eigen::MatrixX3d Raw_data_;
    Eigen::MatrixX3d Single_data;

    // Constructor
    RawMarker(int64_t frame, Eigen::MatrixX3d &Raw_data);

    /* 聚类方法
     * input --> All of points
     * output -->
     */
    void EuclideanCluster(vector<vector<int>> &PotentialCluster_indices);

    bool RadiusSearch(int sq, float radius, vector<bool> &process, vector<int> &k_indices) const;

    /* Decoding
     * input -->
     * output -->
     */
    void AEMC_I_Decoding(int class_indices, vector<int> &cluster_indices, Circle &c, CodedCluster *codedCluster);

    void AEMC_E_Decoding(int class_indices, vector<int> &cluster_indices, CodedCluster *codedCluster);

    void areInMatrix(const vector<int> &, const Eigen::MatrixXf &,
                     vector<pair<vector<int>, vector<int>>> &);

    int bestFit(vector<pair<vector<int>, vector<int>>> &ordered_indices,
                const Eigen::MatrixXf &chordsMatrix);

    /* TODO 模糊匹配
     * 建立四边形的测量模型;
     * 1. 生成所有可能的匹配
     * 2. 生成所有可能的匹配的匹配度
     * 3. 选择最佳匹配
     */

    Vector3d findX();

    /* Fitting
     * input -->
     * output -->
     * 基于RANSAC的圆形滤波
     * 可继续使用处理标志列表
     * 先判断是否处理
     * 1. 采样三个点判断半径是否在设定范围内，
     * 2. 如果在，判断其他点到这个圆的距离是否在范围内，生成一个可解码的聚类，
     * 若不在，更换采样点，重复1，2
     */
    void Fitting(const vector<vector<int>> &PotentialCluster_indices, vector<CodedCluster> &CodedClusterList);

    void Track(vector<PreClusters> &preClusterList, vector<CodedCluster> &curCodedClusterList) const;

    void TraceTrack(vector<PreTrace> &preTrace, vector<CodedCluster> &curCodedClusterList);

    /* The points of this frame and the previous frame are matched
     * using the Hungarian method based on the Euclidean distance.
     * input --> preSingle_data, curSingle_data
     */
    void Hungarian_matching(Eigen::MatrixX3d &preSingle_data);

    /* Use the Hungarian algorithm to rematch cluster lists with confidence levels lower than 0.1.
     * input --> preCodedClusterList, curCodedClusterList
     */
    void Rematch(vector<CodedCluster> preCodedClusterList, vector<CodedCluster> &curCodedClusterList);
};


/* Generate Gaussian distribution confidence
 * input --> mean, variance, x
 * output --> confidence
 */
float Gaussian(float mean, float variance, float x);

/* Get average confidence
 * input --> reference chords, measured chords
 * output --> average confidence
 */
float AverageConfidence(const vector<float> &reference_chords, const vector<float> &measured_chords);

/* Use extended Kalman filter prediction to predict the position information of the next frame
 * input --> pre_position, pre_velocity, pre_acceleration, dt
 * output --> position
 */
void EKF_Predict(Eigen::Vector3d &pre_position, Eigen::Vector3d &pre_velocity, Eigen::Vector3d &pre_acceleration,
                 double dt, Eigen::Vector3d &position);


/* Position-based encoding and decoding operations
 * input --> position, code_length, class_indices
 * output --> codeword
 */
int Position_Encoding(Eigen::Vector3d &position, int code_length, int class_indices);

/* Hungarian algorithm
 * input --> cost matrix
 * output --> assignment
 */
void Hungarian(Eigen::MatrixXf &cost_matrix, vector<int> &assignment);


/* Calculate its quaternion in the world coordinate
 * input --> points in the world coordinate
 * output --> quaternion
 */
Eigen::Quaternion<double> toQuaternion(Eigen::MatrixX3d &pts);


/* Clean the data and filter out points that are very close to each other
 * input --> raw data
 */
void CleanData(Eigen::MatrixX3d &raw_data);

bool filter(vector<Eigen::Vector3d> &pre, Eigen::Vector3d &cur);

bool filter2(PreClusters &pre, Eigen::Vector3d &cur, int64_t frame);

void GenPose(Eigen::MatrixX3d &pts, Eigen::Quaternion<double> &Pose);

void GenTransform(Eigen::MatrixX3d &pts, Eigen::Quaternion<double> &Pose, Eigen::Vector3d &position);

const Circle &Points3Fitting(Circle &cir1, Eigen::Matrix<double, 3, 3> &points);

const Circle &LeastSquareFitting(Eigen::MatrixX3d);

void combine(int N, int M, vector<vector<int>> &com);

bool areRanges(float value, const vector<float> &targets, float error, int &class_idx);

double getCrossAngle(Eigen::Vector3d &vertex, Eigen::Vector3d &P1, Eigen::Vector3d &P2);

/* Reorder the two vectors in a pair according to the second sector
 * input --> pair
 * output --> pair
 */
void reorder(pair<vector<int>, vector<int>> &pair);

bool cmp(const pair<float, vector<int>> &, const pair<float, vector<int>> &);

bool cmp1(const pair<double, int> &a, const pair<double, int> &b);

vector<int> vectors_intersection(vector<int> v1, vector<int> v2);

vector<int> vectors_diff(vector<int> v1, vector<int> v2);

// cyclic shift left
int cyclicShift(int value, int d, int N);

int HammingDistance(int x, int y);

Circle FitCircle(Eigen::MatrixX3d &pts);

#endif //AEMC_CLIENT_AEMC_H
