#include "Circuito.h"

Circuito::Circuito() {}

Circuito::~Circuito()
{
	for (Barra* pBarra : this->barralista)
	{
		delete pBarra;
	}
	for (Ramo* pRamo : this->ramolista)
	{
		delete pRamo;
	}
}

Barra* Circuito::getBarra(int idbrr)
{
	if (this->barralista[idbrr - 1])
	{
		return this->barralista[idbrr - 1];
	}
	return nullptr;
}
