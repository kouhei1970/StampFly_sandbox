#ifndef OPT_KALMAN_HPP
#define OPT_KALMAN_HPP

class Opt_kalman
{
	//float h;

	float p11,p12,p13,p14,p15,p16,p17,p18,p19;
	float p21,p22,p23,p24,p25,p26,p27,p28,p29;
	float p31,p32,p33,p34,p35,p36,p37,p38,p39;
	float p41,p42,p43,p44,p45,p46,p47,p48,p49;
	float p51,p52,p53,p54,p55,p56,p57,p58,p59;
	float p61,p62,p63,p64,p65,p66,p67,p68,p69;
	float p71,p72,p73,p74,p75,p76,p77,p78,p79;
	float p81,p82,p83,p84,p85,p86,p87,p88,p89;
	float p91,p92,p93,p94,p95,p96,p97,p98,p99;

	float _p11,_p12,_p13,_p14,_p15,_p16,_p17,_p18,_p19;
	float _p21,_p22,_p23,_p24,_p25,_p26,_p27,_p28,_p29;
	float _p31,_p32,_p33,_p34,_p35,_p36,_p37,_p38,_p39;
	float _p41,_p42,_p43,_p44,_p45,_p46,_p47,_p48,_p49;
	float _p51,_p52,_p53,_p54,_p55,_p56,_p57,_p58,_p59;
	float _p61,_p62,_p63,_p64,_p65,_p66,_p67,_p68,_p69;
	float _p71,_p72,_p73,_p74,_p75,_p76,_p77,_p78,_p79;
	float _p81,_p82,_p83,_p84,_p85,_p86,_p87,_p88,_p89;
	float _p91,_p92,_p93,_p94,_p95,_p96,_p97,_p98,_p99;

	float q1,q2,q3;
	float r1,r2,r3;

	//rotation matrix
	float r11,r12,r13;
	float r21,r22,r23;
	float r31,r32,r33;


	float beta_x,beta_y,beta_z;

	float s11,s12,s13;
	float s21,s22,s23;
	float s31,s32,s33;

	float si11,si12,si13;
	float si21,si22,si23;
	float si31,si32,si33;

	float k11,k12,k13;
	float k21,k22,k23;
	float k31,k32,k33;
	float k41,k42,k43;
	float k51,k52,k53;
	float k61,k62,k63;
	float k71,k72,k73;
	float k81,k82,k83;
	float k91,k92,k93;

	float e1,e2,e3;
	//Observation
	//z1: u z2:v z3:sens_altitude
	float z1,z2,z3;

	//state
	//x1:u  x2:v  x3:w
	//x4:x  x5:y  x6:z
	//x7:bx x8:by x9:bz
	float x1,x2,x3,x4,x5,x6,x7,x8,x9;
	float _x1,_x2,_x3,_x4,_x5,_x6,_x7,_x8,_x9;
	float asx,asy,asz;
	float kalman_time;

	public:
	Opt_kalman();
	void update(float *accel, float *euler, float *observation, float h);
	void set_state(float* state);
	void get_state(float* state);

};

#endif
