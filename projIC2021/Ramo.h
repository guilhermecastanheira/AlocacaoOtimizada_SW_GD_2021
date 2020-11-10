#pragma once
#include<complex>
#include<map>
#include<string>
#include"Barra.h"

using namespace std;

enum TIPORAMO { NR, SW, CS };	/*	SW: ramo com chave de manobra
										NR:	ramo normal
										CS:	ramo de interconexao entre alimentadores		*/

enum ESTADO { ON, OFF };	/*		ON:		ligado/conectado
									OFF:	desligado/desconectado		*/

class Ramo : public Barra
{
public:
	Ramo();
	~Ramo();

	Barra* pb1;
	Barra* pb2;
	float km;
	complex<float>Zl, Il;

	TIPORAMO tipoRamo;
	TIPORAMO TipoRamo(string idramo);

	ESTADO estadoRamo;
	ESTADO EstadoRamo(string OnOff);

	void setBarra1(Barra* brr);
	void setBarra2(Barra* brr);
};
