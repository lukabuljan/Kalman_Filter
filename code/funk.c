#include <stdint.h>
#include <inttypes.h>
#include <math.h>

#include "funk.h"

#define ACC_NOISE_STD  2.0
#define BAR_NOISE_STD   0.5

double convert_to_rad(float deg) //pretvorba iz stupnjeva u radijane
{
	float k = M_PI/180;
	return deg*k;
}

void interpol(uint32_t time_ref, acc *manji, acc *veci, acc *rez) //f-ja za interpolaciju
{
	float t = (float)time_ref;
	float t0 = (float)manji->time;
	float t1 = (float)veci->time;

	float k = (t-t0)/(t1-t0);
											
	rez->time = time_ref;
	rez->x = manji->x + k*(veci->x - manji->x);
	rez->y = manji->y + k*(veci->y - manji->y);
	rez->z = manji->z + k*(veci->z - manji->z);
}

acc mob_to_fix(acc akcel, acc rotacije)//f-ja za pretvorbu ubrzanja k.s drona u fiksni k.s. s z osi orijentiranoj prema gore
{
	acc convert;

	float phi = convert_to_rad(rotacije.x);
	float theta = convert_to_rad(rotacije.y);
	float psi = convert_to_rad(rotacije.z);

	float mat[3][3] = {{cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi)}, //umnožak rotacijskih matrica oko x,y,z osi za proizvoljne zakrete
			   {sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi)},
			   {-1*sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)}};

	float akc[3] = {akcel.x, akcel.y, akcel.z};

	float prod[3] = {mat[0][0]*akc[0] + mat[0][1]*akc[1] + mat[0][2]*akc[2], mat[1][0]*akc[0] + mat[1][1]*akc[1] + mat[1][2]*akc[2], mat[2][0]*akc[0] + mat[2][1]*akc[1] + mat[2][2]*akc[2]}; //akceleracije u NED k.s.

	convert.time = akcel.time; //konverzija izmjerenih ubrzanja u k.s. gdje je y os lijevo a z os gore orijentirana
	convert.x = prod[0]*9.81;  //dovoljna je dodatna rotacija svih vrijednosti oko osi x za 180 stupnjeva (množenje trenutne matrice slijeva). X komponenta ostaje pozitivna, dok y i z mijenjaju predznak
	convert.y = -prod[1]*9.81;
	convert.z = -prod[2]*9.81 - 9.81;

	return convert;
}

void kalman_init(kalman *s, const bar *first_bar) //inicijalizacija KF varijabli
{
	s->z = first_bar->z;
	s->vz = 0.0;

	s->P00 = BAR_NOISE_STD * BAR_NOISE_STD;
	s->P01 = 0.0;
	s->P10 = 0.0;
	s->P11 = 4.0;

	s->az = 0.0;

	s->current_time = first_bar->time;
}

void kalman_predict(kalman *s, uint32_t next_time) //predikcija na temelju ubrzanja i šuma
{
	float dt = (float)(next_time - s->current_time) * 1e-6;

	float dt2 = dt * dt;
	float dt3 = dt2 * dt;
	float dt4 = dt2 * dt2;

	const float q = ACC_NOISE_STD * ACC_NOISE_STD;

	s->z += s->vz * dt + 0.5 * s->az * dt2;
	s->vz += s->az * dt;

	float nP00 = s->P00 + dt * (s->P01 + s->P10) + dt2 * s->P11 + q * dt4 / 4.0;

	float nP01 = s->P01 + dt * s->P11 + q * dt3 / 2.0;
	float nP10 = s->P10 + dt * s->P11 + q * dt3 / 2.0;
	float nP11 = s->P11 + q * dt2;

	s->P00 = nP00;
	s->P01 = nP01;
	s->P10 = nP10;
	s->P11 = nP11;

	s->current_time = next_time;
}

void kalman_correct(kalman *s, float mjer) //korekcija varijabli KF na temelju pristiglih mjerenja i šuma
{
	const float r = BAR_NOISE_STD * BAR_NOISE_STD;

	float innov = mjer - s->z;
	float S = s->P00 + r;

	float K0 = s->P00 / S;
	float K1 = s->P10 / S;

	float oldP00 = s->P00;
	float oldP01 = s->P01;

	s->z += K0 * innov;
	s->vz += K1 * innov;

	s->P00 = (1.0 - K0) * oldP00;
	s->P01 = oldP01 - K0 * oldP01;
	s->P10 = s->P10 - K1 * oldP00;
	s->P11 = s->P11 - K1 * oldP01;

	float cross = 0.5 * (s->P01 + s->P10);

	s->P01 = cross;
	s->P10 = cross;
}

void kalman_filter(kalman *s, uint32_t t_next, float mjer, acc *akcel, uint8_t acc_or_bar, estimate *est) //varijabla acc_or_bar odreduje radi li se o podatku tj. vremenu iz accelerometer.csv ili iz barometer.csv
{
	kalman_predict(s, t_next);									  //u slučaju podatka iz accelerometer.csv vrsi se predikcija i pohranjuju se varijable za ispis u estimated values.csv 

	if(acc_or_bar)											  //u slučaju podatka iz barometer.csv vrsi se korekcija bez pohrane
	{
		s->az = akcel->z;
		est->time = akcel->time;
		est->z = s->z;
		est->vz = s->vz;
	}
	else{kalman_correct(s, mjer);}
}

