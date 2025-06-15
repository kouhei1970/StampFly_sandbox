#ifndef PID_HPP
#define PID_HPP

class PID
{
  private:
    float m_kp;
    float m_ti;
    float m_td;
    float m_eta;
    float m_err,m_err2,m_err3;
    float m_h;
  public:
    float m_differential;
    float m_integral;
    PID();
    void set_parameter(
        float kp, 
        float ti, 
        float td,
        float eta, 
        float h);
    void reset(void);
    void i_reset(void);
    void printGain(void);
    void set_error(float err);
    float update(float err, float h);
    
    // PIDゲイン取得・設定関数
    float get_kp(void) { return m_kp; }
    float get_ti(void) { return m_ti; }
    float get_td(void) { return m_td; }
    float get_eta(void) { return m_eta; }
    void set_kp(float kp) { m_kp = kp; }
    void set_ti(float ti) { m_ti = ti; }
    void set_td(float td) { m_td = td; }
    void set_eta(float eta) { m_eta = eta; }
};

class Filter
{
  private:
    float m_state;
    float m_T;
    float m_h;
  public:
    float m_out;
    Filter();
    void set_parameter(
        float T,
        float h);
    void reset(void);
    float update(float u, float h);
};

#endif
