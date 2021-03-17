//IC 2020
//ALOCAÇÃO DE CHAVES DE MANOBRA PARA RESTAURAÇÃO

#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <complex>
#include <vector>
#include <algorithm>
#include <tuple>


#include "Funcoes.h"

using namespace std;

// DADOS DO SISTEMA DE 136 BARRAS ---------------------------------------

//Dados para o arquivo txt
#define linha_dados 157 //numero de linhas da matriz de dados +1
#define coluna_dados 9 //numero de colunas da matriz de dados +1

//Caracteristicas Alimentadores e Subestacoes ----------------------------

#define num_AL 9 //numero de alimentadores +1 por conta do cpp
#define estado_restaurativo_pu 0.93 //tensao minima no estado restaurativo
#define capSE 100000000 //potencia maxima de cada alimentador

#define capGD 100000 //potencia de um GD - 10 placas solares

#define CM 1
#define GD 2

int alimentadores[num_AL] = { 0, 1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007 }; // alimentadores

//PARAMETROS FUNCAO OBJETIVO E METAHEURISTICA ------------------------------------

#define tempo_falha 4 //numero de horas que o sistema fica em estado restaurativo
#define tempo_isolacao 0.12 //tempo necessario para fazer as manobras em horas
#define taxa_falhas 0.072 //taxa de falhas por km no ano

#define custoKWh 0.17 //custo kWh fonte não renovavel
#define custoKWhr 0.04 //custo kWh fonte renovavel

#define custoCM 800 //custo de uma chave de manobra
#define custoGD 80000 //custo de um GD despachavel

#define criterio_parada 5 //criterio de parada da metaheuristica
#define numero_simulacoes 30 //numero de simulacoes que o algoritmo faz

//Caracteristicas Fluxo de Potencia ------------------------------------

//valores base, ou, de referencia
float sref = 100 * pow(10, 3); //100MVA
float vref = 13800; //13.8kV
float zref = (vref * vref) / sref;
float iref = sref / vref;

//tensao inicial nos nós do sistema: vref * (valor para dar a tensao inicial)
float pu_inicial_alimentador = 1.05;
float tensao_inicial_nos = (vref)*pu_inicial_alimentador;

//critério de convergencia do fluxo de potencia
complex <float> criterio_conv = 1 * pow(10, -4);
float epsilon = abs(criterio_conv);

int max_interacao = 8;


//NOTAS:
/*
- chaves no estado aberto eh considerado 0;
- chaves fechadas é considerado 1;
- VND com duas vizinhanças de intensificação;
- RVNS com quatro vizinhanças para diversificação;
*/

//#######################################################################################

// CLASSES ---------------------------------------------------

class ParametrosSistema
{
public:

	int noi[linha_dados] = {}; //nó inicial
	int nof[linha_dados] = {}; //nó final

	float lt_r[linha_dados] = {}; //parte real da lt
	float lt_x[linha_dados] = {}; //parte imaginaria da lt

	float s_nofr[linha_dados] = {}; //potencia complexa do nof real
	float s_nofq[linha_dados] = {}; //potencia complexa do nof img

	int candidato_aloc[linha_dados] = {}; //candidato a alocação de chaves
	int estado_swt[linha_dados] = {}; //estado da chave
	int estado_swt_vanila[linha_dados] = {}; //estado das chaves para o sistema vanila

	float dist_no[linha_dados] = {}; //distancia entre nós

	float potencia_al[num_AL] = {}; //potencia de cada alimentador

	float total_ativa = 0;
	complex <float> total_complexa = complex <float>(float(0.0), float(0.0));

	complex <float> lt[linha_dados] = {}; //linha de transmissao entre nós
	complex <float> s_nof[linha_dados] = {}; //potencia complexa do nof

	complex <float> pu_lt[linha_dados] = {}; //linha de transmissao entre nós em pu
	complex <float> pu_s_nof[linha_dados] = {}; //potencia complexa do nof em pu

	void leitura_parametros();
	void somatorio_potencia();

}ps;

class FluxoPotencia
{
public:

	int contadorFXP = 0; //conta quantas vzs realizou o processo de fluxo de potencia

	int camadaREF[linha_dados][linha_dados] = {};

	int conexao_predef[linha_dados][3] = {};

	complex <float> tensao_inicial = complex <float>(float(tensao_inicial_nos / vref), float(0)); // tensao complexa nos nós na 1 iteraçao do fluxo de potencia

	complex <float> corrente_pu[linha_dados] = {};
	complex <float> tensao_pu[linha_dados] = {};

	void valores_nominais_tensao();
	void fluxo_potencia(int referencia);
	bool condicaoTensaoFluxoPotencia(int barra_referencia);

private:

	void camadas(int referencia, int camadailha[linha_dados][linha_dados]);
	void backward_sweep(int referencia);
	void forward_sweep(int referencia);

}fxp;

class DispositivosRede
{
public:
	float potmaxGD = 8 * capGD; //maximo de 80 placas por barra, totalizando 8 blocos de 10
	bool ramo_cm[linha_dados]; //aqui se tem os ramos com chaves de manobra
	bool barra_gd[linha_dados]; //barras com GD
	float cap_barraGD[linha_dados]; //quantidade de potencia na barra pelo GD

	vector<int> refGDAL = { 1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007 }; // alimentadores e GDs - estes sao referencias para solucao do fluxo de potencia, e, inicialmente se coloca as barras que sao alimentadores, nas quais sao as denotadas por serem maiores que 1000

} dr;

class AlocacaoDispositivosRede
{
public:
	vector<vector<int>> secoes;

	void sortCMGD(bool cm_gd[linha_dados], int AL, int tipo); //sorteio de chaves de manobra ou gd

} ad;

class OperacaoSistema
{
public:
	bool ilha;
	bool remanejamento;

	bool analise_operacao(vector<int> secao, int al);
	void isolaSecao(vector<int> s, int intervalo1, int intervalo2);
} os;

class FuncaoObjetivo
{
public:
	float funcaoObjetivo = 0.0;

	float pGD[linha_dados];
	bool lGD[linha_dados];
	bool lCM[linha_dados];

	void retorno(float potenciaGD[linha_dados], bool localGD[linha_dados], bool localCM[linha_dados]);
	void salva(float potenciaGD[linha_dados], bool localGD[linha_dados], bool localCM[linha_dados]);
	float calculo(int AL);
} fo;

//------------------------------------------------------------

void ParametrosSistema::leitura_parametros()
{
	FILE* arquivo;

	if ((arquivo = fopen("dados135.txt", "r")) == NULL)
		return;

	for (int i = 1; i < linha_dados; i++)
	{
		fscanf(arquivo, "%d%d%f%f%f%f%d%d%f", &ps.noi[i], &ps.nof[i], &ps.lt_r[i], &ps.lt_x[i], &ps.s_nofr[i], &ps.s_nofq[i], &ps.candidato_aloc[i], &ps.estado_swt_vanila[i], &ps.dist_no[i]);
	}

	fclose(arquivo);

	//dados complexos

	for (int j = 1; j < linha_dados; j++)
	{
		//dos ramos 

		ps.lt[j].real(ps.lt_r[j]);
		ps.lt[j].imag(ps.lt_x[j]);

		//das potencias complexas

		ps.s_nof[j].real(ps.s_nofr[j] * 1000); //a multiplicação por mil é porque os dados estao em kW
		ps.s_nof[j].imag(ps.s_nofq[j] * 1000); //a multiplicação por mil é porque os dados estao em kVAr
	}

	//dados em pu

	for (int k = 1; k < linha_dados; k++)
	{
		//dos ramos 
		ps.pu_lt[k] = ps.lt[k] / zref;

		//das potencias complexas
		ps.pu_s_nof[k] = ps.s_nof[k] / sref;
	}


	//estado das chaves
	for (int k = 1; k < linha_dados; k++)
	{
		ps.estado_swt[k] = ps.estado_swt_vanila[k];
	}
}

void ParametrosSistema::somatorio_potencia()
{
	//aqui se soma a potencia ativa total, complexa e ativa

	for (int i = 1; i < linha_dados; i++)
	{
		ps.total_ativa = ps.total_ativa + ps.s_nofr[i];
		ps.total_complexa = ps.total_complexa + ps.s_nof[i];
	}
	;
}

void FluxoPotencia::camadas(int referencia, int camadailha[linha_dados][linha_dados])
{
	//zerar camada 
	bool add = false;

	for (int i = 1; i < linha_dados; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			camadailha[i][j] = 0;
		}
	}

	//define o primeiro termo da camada
	camadailha[1][1] = referencia;

	int x = 2;
	int y = 1;


	add = false;
	//montando matriz adjacente

	for (int i = 1; i < linha_dados; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (j != 0)
			{
				for (int k = 1; k < linha_dados; k++)
				{
					add = false;

					if (ps.noi[k] == camadailha[i][j])
					{
						if (ps.estado_swt[k] == 1)
						{
							for (int m = 1; m < linha_dados; m++)
							{
								for (int p = 1; p < linha_dados; p++)
								{
									if (ps.nof[k] == camadailha[m][p])
									{
										add = true; //ja tem o elemento na matriz
									}
								}
							}

							if (add == false)
							{
								camadailha[x][y] = ps.nof[k];
								y++;
							}

						}
					}
				}
				//

				for (int k = 1; k < linha_dados; k++)
				{
					add = false;

					if (ps.nof[k] == camadailha[i][j])
					{
						if (ps.estado_swt[k] == 1)
						{
							for (int m = 1; m < linha_dados; m++)
							{
								for (int p = 1; p < linha_dados; p++)
								{
									if (ps.noi[k] == camadailha[m][p])
									{
										add = true; //ja tem o elemento na matriz
									}
								}
							}

							if (add == false)
							{
								camadailha[x][y] = ps.noi[k];
								y++;
							}
						}
					}
				}
			}
		}
		x++;
		y = 1;
	}
}

void FluxoPotencia::backward_sweep(int referencia)
{
	int posi = 0;
	int posf = 0;

	//atribuição das correntes nas barras
	for (int i = 1; i < linha_dados; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			for (int k = 1; k < linha_dados; k++)
			{
				if (fxp.camadaREF[i][j] == ps.nof[k])
				{
					corrente_pu[k] = conj(ps.pu_s_nof[k] / tensao_pu[k]);
				}
			}
		}
	}

	// somatorio das correntes nos ramos
	for (int i = linha_dados - 1; i > 2; i--) //o i aqui vai ate 1 pq a linha 1 é a do referencia e i-1 sera a linha 1
	{
		for (int k = linha_dados - 1; k > 0; k--)
		{
			if (fxp.camadaREF[i][k] != 0)
			{
				for (int r = 1; r < linha_dados; r++)
				{
					if (fxp.camadaREF[i - 1][r] == 0) { continue; }

					for (int j = 1; j < linha_dados; j++)
					{
						if (fxp.camadaREF[i][k] == ps.nof[j] && fxp.camadaREF[i - 1][r] == ps.noi[j])
						{
							posf = 0;
							posi = 0;

							for (int o = 1; o < linha_dados; o++)
							{
								if (ps.nof[o] == fxp.camadaREF[i - 1][r] && ps.estado_swt[o] == 1)
								{
									if (ps.candidato_aloc[o] == 1 || ps.noi[o] == referencia)
									{
										posf = o;
										break;
									}
								}
							}

							for (int o = 1; o < linha_dados; o++)
							{
								if (ps.nof[o] == fxp.camadaREF[i][k] && ps.estado_swt[o] == 1)
								{
									if (ps.candidato_aloc[o] == 1 || ps.noi[o] == referencia)
									{
										posi = o;
										break;
									}
								}
							}

							fxp.corrente_pu[posf] = fxp.corrente_pu[posf] + fxp.corrente_pu[posi];
						}
						else if (fxp.camadaREF[i][k] == ps.noi[j] && fxp.camadaREF[i - 1][r] == ps.nof[j])
						{
							posf = 0;
							posi = 0;

							for (int o = 1; o < linha_dados; o++)
							{
								if (ps.nof[o] == fxp.camadaREF[i - 1][r] && ps.estado_swt[o] == 1)
								{
									if (ps.candidato_aloc[o] == 1 || ps.noi[o] == referencia)
									{
										posf = o;
										break;
									}
								}
							}

							for (int o = 1; o < linha_dados; o++)
							{
								if (ps.nof[o] == fxp.camadaREF[i][k] && ps.estado_swt[o] == 1)
								{
									if (ps.candidato_aloc[o] == 1 || ps.noi[o] == referencia)
									{
										posi = o;
										break;
									}
								}
							}

							fxp.corrente_pu[posf] = fxp.corrente_pu[posf] + fxp.corrente_pu[posi];
						}
					}
				}

			}
		}
	}

}

void FluxoPotencia::forward_sweep(int referencia)
{

	complex <float> unit = fxp.tensao_inicial; //complexo unitario em pu para a tensao
	int vf = 0;
	int vi = 0;

	//atribuindo a tensao para o restante dos nós

	for (int i = 1; i < (linha_dados - 2); i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (fxp.camadaREF[i][j] == 0) { continue; } //continua se for 0 para adiantar o processo

			for (int k = 1; k < linha_dados; k++)
			{
				if (fxp.camadaREF[i + 1][k] == 0) { continue; }

				for (int t = 1; t < linha_dados; t++)
				{
					if (fxp.camadaREF[i][j] == referencia)
					{
						vf = 0;
						for (int r = 1; r < linha_dados; r++)
						{
							if (ps.noi[r] == referencia)
							{
								vf = r;
								break;
							}
						}

						fxp.tensao_pu[vf] = unit - (fxp.corrente_pu[vf] * ps.pu_lt[vf]);
						break;
					}
					else if (fxp.camadaREF[i][j] == ps.noi[t] && fxp.camadaREF[i + 1][k] == ps.nof[t] && fxp.camadaREF[i][j] != referencia)
					{
						vf = 0;
						vi = 0;

						for (int r = 1; r < linha_dados; r++)
						{
							if (ps.nof[r] == fxp.camadaREF[i][j] && ps.estado_swt[r] == 1)
							{
								vi = r;
								break;
							}
						}

						for (int r = 1; r < linha_dados; r++)
						{
							if (ps.nof[r] == fxp.camadaREF[i + 1][k] && ps.estado_swt[r] == 1)
							{
								vf = r;
								break;
							}
						}

						fxp.tensao_pu[vf] = fxp.tensao_pu[vi] - (fxp.corrente_pu[vf] * ps.pu_lt[t]);

					}

					else if (fxp.camadaREF[i][j] == ps.nof[t] && fxp.camadaREF[i + 1][k] == ps.noi[t] && fxp.camadaREF[i][j] != referencia)
					{
						vf = 0;
						vi = 0;

						for (int r = 1; r < linha_dados; r++)
						{
							if (ps.nof[r] == fxp.camadaREF[i][j] && ps.estado_swt[r] == 1)
							{
								vi = r;
								break;
							}
						}

						for (int r = 1; r < linha_dados; r++)
						{
							if (ps.nof[r] == fxp.camadaREF[i + 1][k] && ps.estado_swt[r] == 1)
							{
								vf = r;
								break;
							}
						}

						fxp.tensao_pu[vf] = fxp.tensao_pu[vi] - (fxp.corrente_pu[vf] * ps.pu_lt[t]);
					}
				}
			}
		}
	}

}

void FluxoPotencia::valores_nominais_tensao()
{
	//PARA ANALISAR O FLUXO

	cout << "\n\n";
	cout << "Fluxo de Potencia \n";
	cout << "No:" << "\t";
	cout << "V:" << "\t";
	cout << "ang:" << "\n";

	/*
	cout << "\t";
	cout << "I:" << "\t";
	cout << "ang:" << "\t";
	cout << "\n";
	*/

	for (int i = 1; i < linha_dados; i++)
	{

		cout << ps.nof[i] << "\t";
		cout << abs(fxp.tensao_pu[i] * vref) << "\t";
		cout << ((arg(fxp.tensao_pu[i])) * 180.0 / 3.14) << "\n";

		/*
		cout << abs(fxp.corrente_pu[i] * iref) << "\t";
		cout << arg(fxp.corrente_pu[i]) * 180 / 3.141592 << "\n";
		*/
	}
}

void FluxoPotencia::fluxo_potencia(int referencia) //alterar conforme o numero de alimentadores, modificando quantas vezes cada funcao eh chamada
{
	bool criterio_satisfeito = false;

	complex <float> tensao_aux[linha_dados];
	complex <float> corrente_aux[linha_dados];

	complex <float> convergencia_tensao[linha_dados];

	int som = 0;
	int iteracao = 0;

	//valores iniciais
	for (int i = 1; i < linha_dados; i++)
	{
		tensao_aux[i] = tensao_inicial;
		corrente_aux[i] = 0.0;

		fxp.tensao_pu[i] = tensao_inicial;
		fxp.corrente_pu[i] = 0.0;
	}

	//definir as camada de fxp.camadas de cada referencia
	camadas(referencia, fxp.camadaREF);

	//estado das chaves
	for (int k = 1; k < linha_dados; k++)
	{
		ps.estado_swt[k] = ps.estado_swt_vanila[k];
	}

	//FLUXO DE POTENCIA

	iteracao = 0;

	while (criterio_satisfeito == false)
	{
		som = 0;
		iteracao += 1;

		//copiando valores anteriores
		for (int i = 1; i < linha_dados; i++)
		{
			tensao_aux[i] = fxp.tensao_pu[i];
		}

		//1 passo: BACKWARD

		//corrente nos ramos
		backward_sweep(referencia);

		//2 passo: FORWARD

		//tensoes nodais
		forward_sweep(referencia);

		//comparação de critério satisfeito

		for (int k = 1; k < linha_dados; k++)
		{
			convergencia_tensao[k] = fxp.tensao_pu[k] - tensao_aux[k];
		}

		for (int k = 1; k < linha_dados; k++)
		{
			if (abs(convergencia_tensao[k]) >= epsilon)
			{
				som += 1;
			}
		}

		if (som == 0 /*|| iteracao == max_interacao*/)
		{
			criterio_satisfeito = true;
		}

	}

	//cout << " max it:     " << iteracao << endl;

	fxp.contadorFXP++; //conta o numero de vzs do processo do fluxo de potencia

}

bool FluxoPotencia::condicaoTensaoFluxoPotencia(int barrareferencia)
{
	bool resultado;
	int locBarra = 0;
	float somaPotencia = 0.0;

	//nesta etapa apenas retorna um valor verdadeiro ou falso para simplificar codigos adiante, mas nao passa do calculo do fluxo de potencia
	fluxo_potencia(barrareferencia);

	//checando condicao
	resultado = true;

	//checando tensao nas barras
	for (int i = 1; i < linha_dados; i++)
	{
		if (abs(fxp.tensao_pu[i]) < estado_restaurativo_pu)
		{
			resultado = false;
		}
	}

	return (resultado);
}

void AlocacaoDispositivosRede::sortCMGD(bool cm_gd[linha_dados], int AL, int tipo)
{
	//sorteia os ramos que podem ser alocados as chaves de manobra

	int a, b;
	int s = 0;
	bool verif = false;

	switch (tipo)
	{
	case CM:
		tie(a, b) = localizaSecao(ps.noi, AL, linha_dados);

		//adicionando a chave
		while (!verif)
		{
			s = rand() % (b - a + 1) + a; //gera um numero entre 'a' e 'b'

			if (cm_gd[s] == false && ps.candidato_aloc[s] == 1 && ps.noi[s] != AL)
			{
				cm_gd[s] = true;
				verif = true;
			}
			else
			{
				verif = false;
			}

		}
		break;

	case GD:
		tie(a, b) = localizaSecao(ps.noi, AL, linha_dados);

		//adicionando gd
		while (!verif)
		{
			s = rand() % (b - a + 1) + a; //gera um numero entre 'a' e 'b'

			if (ps.candidato_aloc[s] == 1)
			{
				cm_gd[s] = true;
				dr.cap_barraGD[s] = dr.cap_barraGD[s] + capGD;

				if (dr.cap_barraGD[s] > dr.potmaxGD)
				{
					dr.cap_barraGD[s] = dr.cap_barraGD[s] - capGD;
					verif = false;
				}
				else {
					verif = true;
				}

			}
		}
		break;
	}
}

void OperacaoSistema::isolaSecao(vector<int>s, int intervalo1, int intervalo2)
{
	//desligando as cheves desta secao

	int posicao = 0;

	for (int i = intervalo1; i <= intervalo2; i++)
	{
		if (dr.ramo_cm[i])
		{
			posicao = posicaoBarra(ps.noi[i], ps.nof, linha_dados);

			for (int k = 0; k < s.size(); k++)
			{
				if (posicao == s[k] || i == s[k])
				{
					ps.estado_swt[i] = DESLIGADO; //abre a chave
				}
			}
			
		}
	}
}

bool OperacaoSistema::analise_operacao(vector<int>secao, int al)
{
	int auxiliar = 0;
	bool cenario = false;
	bool ok = false;
	vector<int> posicao;
	int SEremanejamento = 0;
	float somaPotencia = 0;
	int locBarra = 0;

	//CENARIO 1: analisa a secao para ver se tem gd ou alimentador
	for (int i = 0; i < secao.size(); i++)
	{
		auxiliar = posicaoBarra(secao[i], ps.nof, linha_dados); //posicao

		if (dr.barra_gd[auxiliar] == true)
		{
			//quer dizer q na secao tem pelo menos um GD ou esta conectado a subestacao, assim, coloca-se este na referencia e faz o fluxo de potencia

			cenario = fxp.condicaoTensaoFluxoPotencia(ps.nof[auxiliar]); //retorna verdadeiro

			if (cenario)
			{
				//checando capacidade de potencia
				for (int i = 1; i < linha_dados; i++)
				{
					for (int j = 1; j < linha_dados; j++)
					{
						if (fxp.camadaREF[i][j] != 0)
						{
							locBarra = posicaoBarra(fxp.camadaREF[i][j], ps.nof, linha_dados);
							somaPotencia += ps.s_nofr[locBarra]; //soma a potencia ativa
						}
					}
				}

				if (somaPotencia > dr.cap_barraGD[auxiliar])
				{
					cenario = false;
				}
			}

		}
		else if (ps.noi[auxiliar] == al)
		{
			cenario = fxp.condicaoTensaoFluxoPotencia(ps.noi[auxiliar]);
		}

		if (cenario) {
			return cenario;
		}
	}

	//CENARIO 2: nao há como operar em ilha, logo, efetua-se o chaveamento para remanejar energia de outro alimentador
	posicao.clear();
	for (int i = 0; i < secao.size(); i++)
	{
		posREMANEJAMENTO(&posicao, secao[i], ps.noi, ps.nof, linha_dados); //procura as posicoes que da pra efetuar oremanejamento nesta secao, testar ate que as condicoes sejam satisfeitas
	}
	for (int k = 0; k < posicao.size(); k++)
	{
		ps.estado_swt[posicao[k]] = LIGADO; //fecha a chave
		SEremanejamento = locSE(ps.noi[posicao[k]], ps.nof[posicao[k]], ps.noi, ps.nof, secao); //localiza oalimentador adjacente
		cenario = fxp.condicaoTensaoFluxoPotencia(SEremanejamento); //analisa o fluxo de potencia
		ps.estado_swt[posicao[k]] = DESLIGADO; //abre a chave

		if (cenario) {
			return cenario;
		}
	}

	return cenario;
}

void FuncaoObjetivo::retorno(float potenciaGD[linha_dados], bool localGD[linha_dados], bool localCM[linha_dados])
{
	for (int i = 1; i < linha_dados; i++)
	{
		potenciaGD[i] = fo.pGD[i];
		localGD[i] = fo.lGD[i];
		localCM[i] = fo.lCM[i];
	}
}

void FuncaoObjetivo::salva(float potenciaGD[linha_dados], bool localGD[linha_dados], bool localCM[linha_dados])
{
	for (int i = 1; i < linha_dados; i++)
	{
		fo.pGD[i] = potenciaGD[i];
		fo.lGD[i] = localGD[i];
		fo.lCM[i] = localCM[i];
	}
}

float FuncaoObjetivo::calculo(int AL)
{
	vector<int> aux; //vetor auxiliar

	int i1, i2;
	i1 = i2 = 0; //intervalo que esta o alimentador

	tie(i1, i2) = localizaSecao(ps.noi, AL, linha_dados); //intervalo do alimentador

	//pegando as posicoes das barras e dividindo as secoes deste alimentador - as secoes sao divididas pelas chaves de  manobra que devem ser desligadas
	for (int i = i1; i <= i2; i++)
	{
		if (dr.ramo_cm[i])
		{
			ad.secoes.push_back(aux);
			aux.clear();
			aux.push_back(i);
		}
		else
		{
			aux.push_back(i);
		}
	}

	ad.secoes.push_back(aux);
	aux.clear();

	//reajustando as potencias para o fluxo de potencia - os gds fazem com que parte da demanda de uma barra seja suprida, assim, se o GD gerar mais energia do que a barra consome, a potencia ativa sera negativa, o que causa uma diminuicao de corrente
	for (int i = i1; i <= i2; i++)
	{
		if (dr.barra_gd[i])
		{
			ps.pu_s_nof[i] = ps.pu_s_nof[i] - (dr.cap_barraGD[i] / sref); //potencia na barra com o GD		
		}
	}

	//agora tem que falhar a secao e ver as possibilidades para o remanejamento e/ou operacao em ilhas
	bool checkcamada = false;
	int cont = 0;
	bool funcionamento = false;

	for (int i = 0; i < ad.secoes.size(); i++)
	{
		os.isolaSecao(ad.secoes[i], i1, i2);

		for (int j = 0; j < ad.secoes.size(); j++)
		{
			if (i != j)
			{
				if (checkcamada)
				{
					cont = comparacaoCAMADASECAO(fxp.camadaREF, ad.secoes[j], linha_dados);

					if (cont == ad.secoes[j].size())
					{
						break;
					}
				}

				checkcamada = true;

				funcionamento = os.analise_operacao(ad.secoes[j], AL);

				if (funcionamento)
				{

				}
			}
		}

		voltaEstadoVanila(ps.estado_swt, ps.estado_swt_vanila, linha_dados);
	}

	return funcaoObjetivo;
}




int main()
{
	srand(static_cast <unsigned int> (time(NULL)));	//faz a aleatoriedade com base no relogio
	//srand(time(NULL));

	//leitura dos parametros - le o txt com os dados do problema
	ps.leitura_parametros();

	//soma das potencias totais do sistema
	ps.somatorio_potencia();

	//faz o fluxo de potencia para o sistema vanila - aqui ainda nao se tem os GDs alocados
	for (int i = 0; i < dr.refGDAL.size(); i++)
	{
		fxp.fluxo_potencia(dr.refGDAL[i]);
	}


	fxp.valores_nominais_tensao(); //impreme os valores do sistema vanila

	//inicialmente, deve-se realizar a alocação de chaves de manobra e GD despachaveis de forma aleatória
	for (int i = 1; i < num_AL; i++)
	{
		//dois GD
		ad.sortCMGD(dr.barra_gd, alimentadores[i], GD);
		ad.sortCMGD(dr.barra_gd, alimentadores[i], GD);

		//duas CM
		ad.sortCMGD(dr.ramo_cm, alimentadores[i], CM);
		ad.sortCMGD(dr.ramo_cm, alimentadores[i], CM);
	}

	//com a condição inicial, desenvolve-se a metaheuristica GVNS
	/*

	1 - fazer a operação de ilha no caso de falha

	2 - se nao der certo, fazer o remanejamento

	NOTA:
			- GD são aumentados ou acrescentados
			- CM são movimentadas ou acrescentadas
	*/

	for (int i = 1; i < num_AL; i++)
	{
		fo.funcaoObjetivo = fo.calculo(alimentadores[i]);
	}

	return 0;
}