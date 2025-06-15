#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <cmath>
#include <cstring>
#include <esp_log.h>

// 3x3行列クラス
class Matrix3x3 {
public:
    Matrix3x3();
    Matrix3x3(float m00, float m01, float m02,
              float m10, float m11, float m12,
              float m20, float m21, float m22);
    
    // コピーコンストラクタ
    Matrix3x3(const Matrix3x3& other);
    
    // 代入演算子
    Matrix3x3& operator=(const Matrix3x3& other);
    
    // 行列の加算
    Matrix3x3 operator+(const Matrix3x3& other) const;
    
    // 行列の減算
    Matrix3x3 operator-(const Matrix3x3& other) const;
    
    // 行列の乗算
    Matrix3x3 operator*(const Matrix3x3& other) const;
    
    // スカラー倍
    Matrix3x3 operator*(float scalar) const;
    
    // 転置行列
    Matrix3x3 transpose() const;
    
    // 逆行列
    Matrix3x3 inverse() const;
    
    // 行列式
    float determinant() const;
    
    // 単位行列の設定
    void setIdentity();
    
    // 要素へのアクセス
    float& operator()(int row, int col);
    const float& operator()(int row, int col) const;
    
    // 行列の要素を配列として取得
    const float* data() const;
    
private:
    float m[9]; // 行列の要素（行優先）
};

// 3次元ベクトルクラス
class Vector3 {
public:
    Vector3();
    Vector3(float x, float y, float z);
    
    // コピーコンストラクタ
    Vector3(const Vector3& other);
    
    // 代入演算子
    Vector3& operator=(const Vector3& other);
    
    // ベクトルの加算
    Vector3 operator+(const Vector3& other) const;
    
    // ベクトルの減算
    Vector3 operator-(const Vector3& other) const;
    
    // スカラー倍
    Vector3 operator*(float scalar) const;
    
    // 内積
    float dot(const Vector3& other) const;
    
    // 外積
    Vector3 cross(const Vector3& other) const;
    
    // ノルム（長さ）
    float norm() const;
    
    // 正規化
    Vector3 normalized() const;
    
    // 要素へのアクセス
    float& operator()(int index);
    const float& operator()(int index) const;
    
    // 要素へのアクセス（x, y, z）
    float& x();
    float& y();
    float& z();
    const float& x() const;
    const float& y() const;
    const float& z() const;
    
    // ベクトルの要素を配列として取得
    const float* data() const;
    
private:
    float v[3]; // ベクトルの要素
};

// 行列とベクトルの乗算
Vector3 operator*(const Matrix3x3& mat, const Vector3& vec);

// 楕円体フィッティングのためのユーティリティ関数
namespace EllipsoidFit {
    // 楕円体フィッティングを行う関数
    // points: 入力データ点の配列
    // num_points: データ点の数
    // center: 楕円体の中心（出力）
    // radii: 楕円体の半径（出力）
    // evecs: 楕円体の主軸（出力、3x3行列）
    bool fit(const Vector3* points, int num_points, 
             Vector3& center, Vector3& radii, Matrix3x3& evecs);
    
    // 最小二乗法による楕円体フィッティング
    bool leastSquaresFit(const Vector3* points, int num_points,
                         Vector3& center, Vector3& radii, Matrix3x3& evecs);
}

#endif // MATRIX_HPP
