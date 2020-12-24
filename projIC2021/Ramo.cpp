#include "Ramo.h"

map<string, TIPORAMO> mapRamo
{
	{"1", NR},
	{"2", SW},
	{"3", CS}
};

map<string, ESTADO> mapEstado
{
	{"1", ON},
	{"0", OFF}
};

//construtor do ramo
Ramo::Ramo()
	:pb1(nullptr), pb2(nullptr), Il(0.0f, 0.0f), Zl(0.0f, 0.0f), km(0.0)
{ }

//destrutor - apaga ponteiros
Ramo::~Ramo()
{ }

//define o tipo do ramo
TIPORAMO Ramo::TipoRamo(string idramo)
{
	return tipoRamo = mapRamo[idramo];
}

//define o estado do ramo
ESTADO Ramo::EstadoRamo(string OnOff)
{
	return estadoRamo = mapEstado[OnOff];
}

//seleciona barra1 - no inicial 
void Ramo::setBarra1(Barra* brr)
{
	this->pb1 = brr;
}

//seleciona barra2 - no final
void Ramo::setBarra2(Barra* brr)
{
	this->pb2 = brr;
}
