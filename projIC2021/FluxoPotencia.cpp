#include "FluxoPotencia.h"

FluxoPotencia::FluxoPotencia(Circuito* circuit)
	: pCirc(circuit), tolerancia(0.00001), tensaoInicial(0.0f, 0.0f)
{ }

FluxoPotencia::~FluxoPotencia()
{ }

