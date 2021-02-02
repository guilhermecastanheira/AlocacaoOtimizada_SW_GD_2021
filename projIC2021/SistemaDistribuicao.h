#pragma once

#include "Circuito.h"
#include <fstream>
#include <iostream>

enum ELEMENTO { BARRA, RAMO, SUB, nSUB }; //barra - ramo - subestacao - nao subestacao

class SistemaDistribuicao : public Circuito
{
private:
	bool elementofim(string ler);
	ELEMENTO NomeElemento(string nome);
	ELEMENTO VerifSUB(string sub);

public:
	vector<vector<Ramo*>>ss; //vetor de subsistemas 
	vector<Ramo*>interlig; //vetor de interconexao dos alimentadores 

	Circuito* DadosSistema(string arq); //leitura dos dados do sistema
	
	void Topologia(Circuito* pc);
};

