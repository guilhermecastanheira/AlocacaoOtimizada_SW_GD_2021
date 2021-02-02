#pragma once

#include<string>
#include<complex>
#include<map>

using namespace std;

enum TIPOBARRA { N, GD, SE };	/*		N:barra normal
										GD: barra com GD
										SE: subestacao		*/

class Barra
{
public:
	Barra();
	~Barra();

	int id;
	complex<float>Vbus, Ibus, Sbus, Sgen;

	TIPOBARRA tipoBarra;
	TIPOBARRA TipoBarra(string idbarra);

};

