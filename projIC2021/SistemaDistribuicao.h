#pragma once

#include "Circuito.h"
#include <fstream>
#include <iostream>

enum ELEMENTO { BARRA, RAMO };

class SistemaDistribuicao : public Circuito
{
private:
	bool elementofim(string ler);
	ELEMENTO NomeElemento(string nome);

public:
	Circuito* DadosSistema(string arq);
};

