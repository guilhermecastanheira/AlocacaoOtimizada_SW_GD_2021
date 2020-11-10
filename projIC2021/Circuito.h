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

	vector<Ramo*>ramolista;
	vector<Barra*>barralista;

	Barra* getBarra(int idbrr);

};

