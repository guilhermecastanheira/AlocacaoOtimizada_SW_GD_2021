#include "FluxoPotencia.h"

FluxoPotencia::FluxoPotencia(Circuito* circuit, const float ref_tol, complex<float>vinicial, const float vbase, const float sbase)
	: pCirc(circuit), tolerancia(ref_tol), tensaoInicial(vinicial), Vb(vbase), Sb(sbase)
{ }

FluxoPotencia::~FluxoPotencia()
{ }

void FluxoPotencia::inicializar()
{
	//atribuir tensao nas barras
	for (Barra* pb : this->pCirc->ramolista)
	{
		pb->Vbus = this->tensaoInicial;
	}
}

void FluxoPotencia::resultado()
{
	this->inicializar();
}

