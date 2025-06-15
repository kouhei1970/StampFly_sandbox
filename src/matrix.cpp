#include "matrix.hpp"
#include <cmath>

// Matrix3x3クラスの実装

Matrix3x3::Matrix3x3() {
    setIdentity();
}

Matrix3x3::Matrix3x3(float m00, float m01, float m02,
                     float m10, float m11, float m12,
                     float m20, float m21, float m22) {
    m[0] = m00; m[1] = m01; m[2] = m02;
    m[3] = m10; m[4] = m11; m[5] = m12;
    m[6] = m20; m[7] = m21; m[8] = m22;
}

Matrix3x3::Matrix3x3(const Matrix3x3& other) {
    memcpy(m, other.m, 9 * sizeof(float));
}

Matrix3x3& Matrix3x3::operator=(const Matrix3x3& other) {
    if (this != &other) {
        memcpy(m, other.m, 9 * sizeof(float));
    }
    return *this;
}

Matrix3x3 Matrix3x3::operator+(const Matrix3x3& other) const {
    Matrix3x3 result;
    for (int i = 0; i < 9; i++) {
        result.m[i] = m[i] + other.m[i];
    }
    return result;
}

Matrix3x3 Matrix3x3::operator-(const Matrix3x3& other) const {
    Matrix3x3 result;
    for (int i = 0; i < 9; i++) {
        result.m[i] = m[i] - other.m[i];
    }
    return result;
}

Matrix3x3 Matrix3x3::operator*(const Matrix3x3& other) const {
    Matrix3x3 result;
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i * 3 + j] = 0;
            for (int k = 0; k < 3; k++) {
                result.m[i * 3 + j] += m[i * 3 + k] * other.m[k * 3 + j];
            }
        }
    }
    
    return result;
}

Matrix3x3 Matrix3x3::operator*(float scalar) const {
    Matrix3x3 result;
    for (int i = 0; i < 9; i++) {
        result.m[i] = m[i] * scalar;
    }
    return result;
}

Matrix3x3 Matrix3x3::transpose() const {
    Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[j * 3 + i] = m[i * 3 + j];
        }
    }
    return result;
}

float Matrix3x3::determinant() const {
    return m[0] * (m[4] * m[8] - m[5] * m[7]) -
           m[1] * (m[3] * m[8] - m[5] * m[6]) +
           m[2] * (m[3] * m[7] - m[4] * m[6]);
}

Matrix3x3 Matrix3x3::inverse() const {
    float det = determinant();
    if (fabs(det) < 1e-6f) {
        // 行列式がほぼ0の場合は逆行列が存在しない
        ESP_LOGE("MATRIX", "Matrix is singular, cannot compute inverse");
        return Matrix3x3(); // 単位行列を返す
    }
    
    float invDet = 1.0f / det;
    
    Matrix3x3 result;
    result.m[0] = (m[4] * m[8] - m[5] * m[7]) * invDet;
    result.m[1] = (m[2] * m[7] - m[1] * m[8]) * invDet;
    result.m[2] = (m[1] * m[5] - m[2] * m[4]) * invDet;
    result.m[3] = (m[5] * m[6] - m[3] * m[8]) * invDet;
    result.m[4] = (m[0] * m[8] - m[2] * m[6]) * invDet;
    result.m[5] = (m[2] * m[3] - m[0] * m[5]) * invDet;
    result.m[6] = (m[3] * m[7] - m[4] * m[6]) * invDet;
    result.m[7] = (m[1] * m[6] - m[0] * m[7]) * invDet;
    result.m[8] = (m[0] * m[4] - m[1] * m[3]) * invDet;
    
    return result;
}

void Matrix3x3::setIdentity() {
    m[0] = 1.0f; m[1] = 0.0f; m[2] = 0.0f;
    m[3] = 0.0f; m[4] = 1.0f; m[5] = 0.0f;
    m[6] = 0.0f; m[7] = 0.0f; m[8] = 1.0f;
}

float& Matrix3x3::operator()(int row, int col) {
    return m[row * 3 + col];
}

const float& Matrix3x3::operator()(int row, int col) const {
    return m[row * 3 + col];
}

const float* Matrix3x3::data() const {
    return m;
}

// Vector3クラスの実装

Vector3::Vector3() {
    v[0] = 0.0f;
    v[1] = 0.0f;
    v[2] = 0.0f;
}

Vector3::Vector3(float x, float y, float z) {
    v[0] = x;
    v[1] = y;
    v[2] = z;
}

Vector3::Vector3(const Vector3& other) {
    v[0] = other.v[0];
    v[1] = other.v[1];
    v[2] = other.v[2];
}

Vector3& Vector3::operator=(const Vector3& other) {
    if (this != &other) {
        v[0] = other.v[0];
        v[1] = other.v[1];
        v[2] = other.v[2];
    }
    return *this;
}

Vector3 Vector3::operator+(const Vector3& other) const {
    return Vector3(v[0] + other.v[0], v[1] + other.v[1], v[2] + other.v[2]);
}

Vector3 Vector3::operator-(const Vector3& other) const {
    return Vector3(v[0] - other.v[0], v[1] - other.v[1], v[2] - other.v[2]);
}

Vector3 Vector3::operator*(float scalar) const {
    return Vector3(v[0] * scalar, v[1] * scalar, v[2] * scalar);
}

float Vector3::dot(const Vector3& other) const {
    return v[0] * other.v[0] + v[1] * other.v[1] + v[2] * other.v[2];
}

Vector3 Vector3::cross(const Vector3& other) const {
    return Vector3(
        v[1] * other.v[2] - v[2] * other.v[1],
        v[2] * other.v[0] - v[0] * other.v[2],
        v[0] * other.v[1] - v[1] * other.v[0]
    );
}

float Vector3::norm() const {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Vector3 Vector3::normalized() const {
    float n = norm();
    if (n < 1e-6f) {
        return Vector3();
    }
    float invN = 1.0f / n;
    return Vector3(v[0] * invN, v[1] * invN, v[2] * invN);
}

float& Vector3::operator()(int index) {
    return v[index];
}

const float& Vector3::operator()(int index) const {
    return v[index];
}

float& Vector3::x() {
    return v[0];
}

float& Vector3::y() {
    return v[1];
}

float& Vector3::z() {
    return v[2];
}

const float& Vector3::x() const {
    return v[0];
}

const float& Vector3::y() const {
    return v[1];
}

const float& Vector3::z() const {
    return v[2];
}

const float* Vector3::data() const {
    return v;
}

// 行列とベクトルの乗算
Vector3 operator*(const Matrix3x3& mat, const Vector3& vec) {
    return Vector3(
        mat(0, 0) * vec.x() + mat(0, 1) * vec.y() + mat(0, 2) * vec.z(),
        mat(1, 0) * vec.x() + mat(1, 1) * vec.y() + mat(1, 2) * vec.z(),
        mat(2, 0) * vec.x() + mat(2, 1) * vec.y() + mat(2, 2) * vec.z()
    );
}

// 楕円体フィッティングのユーティリティ関数の実装
namespace EllipsoidFit {

bool fit(const Vector3* points, int num_points, 
         Vector3& center, Vector3& radii, Matrix3x3& evecs) {
    return leastSquaresFit(points, num_points, center, radii, evecs);
}

// 最小二乗法による楕円体フィッティング
bool leastSquaresFit(const Vector3* points, int num_points,
                     Vector3& center, Vector3& radii, Matrix3x3& evecs) {
    if (num_points < 9) {
        ESP_LOGE("ELLIPSOID_FIT", "Not enough points for ellipsoid fitting");
        return false;
    }
    
    // 重心を計算
    Vector3 centroid(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < num_points; i++) {
        centroid = centroid + points[i];
    }
    centroid = centroid * (1.0f / num_points);
    
    // 共分散行列を計算
    Matrix3x3 covariance;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            covariance(i, j) = 0.0f;
        }
    }
    
    for (int i = 0; i < num_points; i++) {
        Vector3 p = points[i] - centroid;
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                covariance(j, k) += p(j) * p(k);
            }
        }
    }
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            covariance(i, j) /= num_points;
        }
    }
    
    // 固有値分解（ヤコビ法）
    // 簡易版の実装のため、より高度な実装が必要な場合は改良が必要
    
    // 初期化
    Matrix3x3 V; // 固有ベクトル
    V.setIdentity();
    
    const int MAX_ITERATIONS = 50;
    const float EPSILON = 1e-6f;
    
    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        // 最大の非対角要素を見つける
        int p = 0, q = 1;
        float max_val = fabs(covariance(0, 1));
        
        if (fabs(covariance(0, 2)) > max_val) {
            max_val = fabs(covariance(0, 2));
            p = 0; q = 2;
        }
        
        if (fabs(covariance(1, 2)) > max_val) {
            max_val = fabs(covariance(1, 2));
            p = 1; q = 2;
        }
        
        // 収束判定
        if (max_val < EPSILON) {
            break;
        }
        
        // ヤコビ回転
        float theta = 0.5f * atan2(2.0f * covariance(p, q), 
                                  covariance(p, p) - covariance(q, q));
        float c = cos(theta);
        float s = sin(theta);
        
        Matrix3x3 J;
        J.setIdentity();
        J(p, p) = c; J(p, q) = -s;
        J(q, p) = s; J(q, q) = c;
        
        // 行列の更新
        Matrix3x3 temp = J.transpose() * covariance * J;
        covariance = temp;
        
        // 固有ベクトルの更新
        V = V * J;
    }
    
    // 結果の設定
    center = centroid;
    
    // 固有値（楕円体の半径の二乗）
    radii.x() = sqrt(fabs(covariance(0, 0)));
    radii.y() = sqrt(fabs(covariance(1, 1)));
    radii.z() = sqrt(fabs(covariance(2, 2)));
    
    // 固有ベクトル（楕円体の主軸）
    evecs = V;
    
    return true;
}

} // namespace EllipsoidFit
