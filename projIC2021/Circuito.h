#pragma once

#include<vector>
#include"Barra.h"
#include"Ramo.h"

using namespace std;

class Circuito : public Ramo
{
public:

	Circuito();
	~Circuito();

	vector<Ramo*>ramolista; //monta a lista de ramos em um subsistema
	vector<Barra*>barralista; //monta um vetor de barras

	Barra* getBarra(int idbrr); //localizar o objeto da barra identificada
};

