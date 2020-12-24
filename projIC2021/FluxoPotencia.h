#pragma once

#include "Circuito.h"

class FluxoPotencia : public Circuito 
{
public:
	FluxoPotencia(Circuito* circuit, const float ref_tol, complex<float>vinicial, const float vbase, const float sbase);
	~FluxoPotencia();

	void resultado();
	void inicializar();
	void atualizarperdas();
	void layer();
	void forwardsweep();
	void backwardsweep();


	Circuito* pCirc;
	float tolerancia;
	float Sb;
	float Vb;
	complex<float>tensaoInicial;
	float perdas_anterior;
	float perdas_posterior;
	vector<int>camadas;
	

	


};

