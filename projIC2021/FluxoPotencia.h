#pragma once

#include "SistemaDistribuicao.h"

class FluxoPotencia : public Circuito 
{
public:
	FluxoPotencia(const float ref_tol, complex<float>vinicial, const float vbase, const float sbase);
	~FluxoPotencia();

	void resultado(SistemaDistribuicao* psd);
	void inicializar(vector<vector<Ramo*>>subsistema);
	void bases(vector<vector<Ramo*>>subsistema);
	void anterior(vector<vector<Ramo*>>subsistema);
	void atualizarperdas();
	void layer(vector<vector<Ramo*>>subsistema);
	void forwardsweep();
	void backwardsweep(vector<vector<Ramo*>>subsistema);
	bool analise_tolerancia();

	
	float tolerancia;
	float Sb;
	float Vb;
	complex<float>tensaoInicial;
	float perdas_anterior;
	float perdas_posterior;
	
	vector<complex<float>>S_numerico;

	vector<int>camadas;
	

	


};

