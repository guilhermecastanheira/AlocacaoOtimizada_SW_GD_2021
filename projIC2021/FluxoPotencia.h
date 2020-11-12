#pragma once

#include "Circuito.h"

class FluxoPotencia : public Circuito 
{
public:
	FluxoPotencia(Circuito* circuit);
	~FluxoPotencia();

	const float tolerancia;
	complex<float>tensaoInicial;

	Circuito* pCirc;


};

