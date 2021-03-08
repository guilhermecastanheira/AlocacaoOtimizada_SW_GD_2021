#include "FluxoPotencia.h"

FluxoPotencia::FluxoPotencia(const float ref_tol, complex<float>vinicial, const float vbase, const float sbase)
	: tolerancia(ref_tol), tensaoInicial(vinicial), Vb(vbase), Sb(sbase)
{ }

FluxoPotencia::~FluxoPotencia()
{ }

void FluxoPotencia::inicializar(vector<vector<Ramo*>>subsistema)
{
	//atribuir tensao nas barras
	for (int i = 0; i < subsistema.size(); i++)
	{
		for (int j = 0; j < subsistema[i].size(); j++)
		{
			if (subsistema[i][j]->pb1->tipoBarra == SE)
			{
				subsistema[i][j]->pb1->Vbus = tensaoInicial;
				subsistema[i][j]->pb2->Vbus = tensaoInicial;
			}
			else
			{
				subsistema[i][j]->pb2->Vbus = tensaoInicial;
			}
		}
	}
}

void FluxoPotencia::bases(vector<vector<Ramo*>>subsistema)
{

	//atribuir tensao nas barras
	for (int i = 0; i < subsistema.size(); i++)
	{
		for (int j = 0; j < subsistema[i].size(); j++)
		{
			if (subsistema[i][j]->pb1->tipoBarra == SE)
			{
				subsistema[i][j]->pb1->Vbus = (subsistema[i][j]->pb1->Vbus / Vb);
				subsistema[i][j]->pb2->Vbus = (subsistema[i][j]->pb2->Vbus / Vb);
			}
			else
			{
				subsistema[i][j]->pb2->Vbus = (subsistema[i][j]->pb2->Vbus / Vb);
			}
			
			subsistema[i][j]->Sbus = (subsistema[i][j]->Sbus / Sb);
		}
	}
}

void FluxoPotencia::anterior(vector<vector<Ramo*>>subsistema)
{
	S_numerico.clear();
	int incremento = 0;

	for (int i = 0; i < subsistema.size(); i++)
	{
		for (int j = 0; j < subsistema[i].size(); j++)
		{
			S_numerico[incremento] = (subsistema[i][j]->pb2->Vbus) * conj(subsistema[i][j]->pb2->Ibus); // S = V.I*
		}
	}
}

void FluxoPotencia::layer(vector<vector<Ramo*>>subsistema)
{

}

void FluxoPotencia::backwardsweep(vector<vector<Ramo*>>subsistema)
{
	//soma de correntes 
}

void FluxoPotencia::resultado(SistemaDistribuicao* psd)
{
	//inicializa as tensoes
	this->inicializar(psd->ss);

	//valores base
	this->bases(psd->ss);

	//valores anteriores
	this->anterior(psd->ss);

	//camadas
	this->layer(psd->ss);

	//fluxo de potencia

	bool condicao = false;

	while (condicao == false)
	{
		//valores anteriores
		this->anterior(psd->ss);

		this->backwardsweep(psd->ss); //primeira etapa





	}

	

	
}

