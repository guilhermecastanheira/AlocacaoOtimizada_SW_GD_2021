#include "Barra.h"


//funcao que define se a barra tem GD ou nao
map<string, TIPOBARRA> mapBarra
{
	{"1", N},
	{"2", GD},
	{"3", SE}
};

//construtor da barra
Barra::Barra()
	: id(0), Vbus(0.0f, 0.0f), Ibus(0.0f, 0.0f), Sbus(0.0f, 0.0f), Sgen(0.0f, 0.0f)
{ }

//destrutor padrao
Barra::~Barra()
{ }

//funcao para mapear a barra
TIPOBARRA Barra::TipoBarra(string idbarra)
{
	return tipoBarra = mapBarra[idbarra];
}


