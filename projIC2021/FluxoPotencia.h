#pragma once

#include "Circuito.h"

class FluxoPotencia : public Circuito 
{
public:
	FluxoPotencia();
	~FluxoPotencia();

	const float tolerancia;
	complex<float>tensaoInicial;


};

