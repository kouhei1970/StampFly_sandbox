#ifndef MAG_HPP
#define MAG_HPP

#include <bmm150.h>
#include "matrix.hpp"
#include <esp_log.h>

// 地磁気センサーのキャリブレーションパラメータ
struct MagCalibParams {
    // ハードアイアン補正（オフセット）
    float offset_x;
    float offset_y;
    float offset_z;
    
    // ソフトアイアン補正（スケールファクター）
    float scale_x;
    float scale_y;
    float scale_z;
};

class MagSensor {
public:
    MagSensor();
    
    // 初期化
    bool init();
    
    // 地磁気データの更新
    bool update();
    
    // 楕円体フィッティングによるキャリブレーション
    void calibrateEllipsoid();
    
    // キャリブレーションデータの収集
    void collectCalibrationData();
    
    // キャリブレーション済みの地磁気データの取得
    void getCalibratedData(float &x, float &y, float &z);
    
    // 生の地磁気データの取得
    void getRawData(float &x, float &y, float &z);
    
    // 地磁気の方位角を計算（ラジアン）
    float getHeading(float roll, float pitch);
    
    // キャリブレーションパラメータの保存
    void saveCalibration();
    
    // キャリブレーションパラメータの読み込み
    void loadCalibration();
    
    // キャリブレーションパラメータのリセット
    void resetCalibration();
    
    // キャリブレーションモードの設定
    void setCalibrationMode(bool enable);
    
    // キャリブレーションモードの取得
    bool isCalibrationMode();
    
private:
    BMM150 bmm150;                // BMM150センサーオブジェクト
    MagCalibParams calib_params;  // キャリブレーションパラメータ
    
    float mag_x, mag_y, mag_z;    // 生の地磁気データ
    float cal_x, cal_y, cal_z;    // キャリブレーション済みの地磁気データ
    
    bool calibration_mode;        // キャリブレーションモードフラグ
    
    // キャリブレーションデータ
    static const int MAX_CALIB_SAMPLES = 300;
    int calib_sample_count;
    Vector3 calib_samples[MAX_CALIB_SAMPLES];
    
    // キャリブレーションデータの適用
    void applyCalibration();
    
    // 楕円体フィッティングアルゴリズム
    void ellipsoidFitting();
    
    static const char* TAG;       // ログ用タグ
};

extern MagSensor mag_sensor;      // グローバルなセンサーオブジェクト

#endif // MAG_HPP
