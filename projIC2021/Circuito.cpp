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
	for (int k = 0; k < barralista.size(); k++)
	{
		if (this->barralista[k]->id == idbrr)
		{
			return this->barralista[k];
		}
	}
	
	return nullptr;
}
