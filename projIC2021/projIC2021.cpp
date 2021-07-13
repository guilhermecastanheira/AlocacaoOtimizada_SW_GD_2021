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

using namespace std;

// DADOS DO SISTEMA DE 136 BARRAS ---------------------------------------

//Dados para o arquivo txt
#define linha_dados 157 //numero de linhas da matriz de dados +1
#define coluna_dados 9 //numero de colunas da matriz de dados +1

//Caracteristicas Alimentadores e Subestacoes ----------------------------

#define num_AL 9 //numero de alimentadores +1 por conta do cpp
#define estado_restaurativo_pu 0.93 //tensao minima no estado restaurativo
#define capSE 100000000 //potencia maxima de cada alimentador
#define maxGD 3 //numero maximo de GD que pode ser instalado na barra
#define num_c 8 //numeros de cenarios a serem considerados (+1 para o cpp)

int alimentadores[num_AL] = { 0, 1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007 }; // alimentadores

//PARAMETROS FUNCAO OBJETIVO E METAHEURISTICA ------------------------------------

#define tempo_falha 4 //numero de horas que o sistema fica em estado restaurativo
#define tempo_isolacao 0.12 //tempo necessario para fazer as manobras em horas
#define taxa_falhas 0.00000822 //taxa de falhas por km por hora - faz isso pq no fim a FO se dará em ano
//#define taxa_falhas 0.087
#define GDinicial 2 //numero de gd inicial em cada alimentador
#define SWinicial 2 //numero de chaves de manobra inicial em cada alimentador

#define criterio_parada 15 //criterio de parada da metaheuristica
#define numero_simulacoes 30 //numero de simulacoes que o algoritmo faz

// Custos -------------------------------------

#define custokwh_naorenovavel 0.17 //em dolar
#define custokwh_renovavel 0.05 //em dolar
#define custoGD 4080 //em dolar - gd
#define custoSW 1150 //em dolar - chave de manobra com manutenção

//Caracteristicas Fluxo de Potencia ------------------------------------

//valores base, ou, de referencia
float sref = 100 * pow(10, 6); //100MVA
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

	int candidato_aloc[linha_dados] = {}; //candidato a alocação de chaves - IC ANTIGA
	int candidato_GD[linha_dados] = {}; //candidato a alocação de GDs
	int candidato_CH[linha_dados] = {}; // candidato a alocação de chaves - IC 2021
	int estado_swt[linha_dados] = {}; //estado da chave
	int estado_swt_vanila[linha_dados] = {}; //estado das chaves para o sistema vanila

	float dist_no[linha_dados] = {}; //distancia entre nós

	float potencia_al = 0.0; //potencia de cada alimentador

	float cenario_demanda[num_c];
	float cenario_is[num_c];
	float tempo_cenario[num_c];
	float custokwh[num_c];
	float custotaxa_CO2[num_c];
	float custoENS[num_c];

	float total_ativa = 0;
	complex <float> total_complexa = complex <float>(float(0.0), float(0.0));

	complex <float> lt[linha_dados] = {}; //linha de transmissao entre nós
	complex <float> s_nof[linha_dados] = {}; //potencia complexa do nof

	complex <float> pu_lt[linha_dados] = {}; //linha de transmissao entre nós em pu
	complex <float> pu_s_nof[linha_dados] = {}; //potencia complexa do nof em pu

	void leitura_parametros();
	void leituraCenarios();
	void somatorio_potencia();

	float potenciaalimentador(int barrasAL[linha_dados]);

}ps;

class FluxoPotencia
{
public:

	int contadorFXP = 0; //conta quantas vzs realizou o processo de fluxo de potencia

	int camadaAL[num_AL][linha_dados][linha_dados] = {};

	int camadaREF[linha_dados][linha_dados] = {};

	int conexao_predef[linha_dados][3] = {};

	complex <float> tensao_inicial = complex <float>(float(tensao_inicial_nos / vref), float(0)); // tensao complexa nos nós na 1 iteraçao do fluxo de potencia

	complex <float> corrente_pu[linha_dados] = {};
	complex <float> tensao_pu[linha_dados] = {};

	void valores_nominais_tensao();

	void fluxo_potencia(bool flagCAMADA = false);
	float perdas_ativa(int barras[linha_dados]);

	void fluxo_potencia_simples(int referencia);
	bool condicaoTensaoFluxoPotencia(int barra_referencia);

private:

	void camadas(int alimentador, int camadaalimentador[linha_dados][linha_dados]);
	void backward_sweep(int alimentador, int almt, float demanda);
	void backward_sweep(int alimentador, int almt);
	void forward_sweep(int alimentador, int alt);

	void camadas_simples(int referencia, int camadailha[linha_dados][linha_dados]);
	void backward_sweep_simples(int referencia);
	void forward_sweep_simples(int referencia);

}fxp;

class AlocacaoChaves
{
public:

	int numch_AL[num_AL] = {}; //numero de chaves por alimentador
	int numch_SIS = 0;
	int posicaochaves[num_AL][linha_dados] = {}; //vetor com as posicoes das chaves
	int adjacente_chaves[num_AL][linha_dados][linha_dados] = {}; //secoes de todas as chaves, inclusive do disjuntor do alimentador 
	int secoes_chaves[num_AL][linha_dados][linha_dados] = {}; //secoes das chaves
	int chi[num_AL][linha_dados] = {}; //barra inicial da chave
	int chf[num_AL][linha_dados] = {}; //barra final da chave

	int antchi[num_AL][linha_dados] = {}; //barra inicial da chave
	int antchf[num_AL][linha_dados] = {}; //barra final da chave
	int antpos[num_AL][linha_dados] = {}; //posicao da chave no sistema

	void chaves_anteriores();
	void volta_chaves_anteriores();

	void secoes_alimentador();

	void adjacentes(int posicao[linha_dados], int adj[linha_dados][linha_dados], int alimentador); //calcula os adjacentes das chaves e da secao do alimentador
	float energia_suprida(int AL, vector<int>barras_AL);

}ac;

class AlocacaoGD
{
public:
	
	int numgd_AL[num_AL] = {}; //numero de gds por alimentador
	int posicaoGD[num_AL][linha_dados] = {}; //vetor com as posicoes do GD
	int antGD[num_AL][linha_dados] = {}; //vetor com posicoes do gd anterior
	float GDbarra[linha_dados] = {}; //potencia dos GDs instalada na Barra
	int quantGD[linha_dados] = {}; //quantidade de gd na barra
	int ant_quantGD[linha_dados] = {}; //quantidade anterior de GD na barra
	int qntGD_SIS = 0; //total de GDs no sistema


	bool opILHA(int secoesCM[linha_dados][linha_dados], int adjCM[linha_dados][linha_dados], int posGD[linha_dados], int AL, int secao);

	void gd_anteriores();

	void volta_gd_anteriores();

	void atualizaPosGD();

	float PotW(float IS);

	void ajustePOT(complex<float> S[linha_dados], float P[linha_dados], float P_gd[linha_dados], float demanda, int posGD[linha_dados]);

	void potenciaGD(float ISpv, float PGD[linha_dados], int numGDbarra[linha_dados]);
	void distribuicao_potencia(int secao[linha_dados]);


}agd;

class FuncaoObjetivo
{
public:

	float fo_al[num_AL] = {};
	float fo_al_save[num_AL] = {};

	float investimentoCM = 0.0;
	float investimentoGD = 0.0;
	float custoENS = 0.0;
	float custoPe = 0.0;
	float custoP = 0.0;
	float custoEmss = 0.0;

	float eCO2 = 0.92; //taxa de emissao de carbono kgCO2/kWh

	float calculo_funcao_objetivo(bool all_al, int p_al);

	float FO(float potenciaAL, float comprimento, float ens);
}fo;

class GVNS
{
public:

	int antchivnd[num_AL][linha_dados] = {}; //barra inicial da chave
	int antchfvnd[num_AL][linha_dados] = {}; //barra final da chave
	int antposvnd[num_AL][linha_dados] = {}; //posicao da chave no sistema

	float fo_al_savevnd[num_AL] = {};

	int localizaAL(int posicao);

	void primeiraaloc();
	void sorteiochaves(int numch, int camada[linha_dados][linha_dados], int posicao_camada[linha_dados], int alimentador); //sorteio inicial das chaves
	void sorteioGDs(int numgd, int camada[linha_dados][linha_dados], int posicao_camada[linha_dados], int alimentador, int qntGD[linha_dados]); //sorteio inicial de GDs
}gvns;

class RVNS : public GVNS
{
public:

	int q_rvns1 = 0;
	int q_rvns2 = 0;
	int q_rvns3 = 0;
	int q_rvns4 = 0;
	int q_rvns5 = 0;

	float v1_RVNS(float incumbentmain1);
	float v2_RVNS_AUX(float incumbentmain2, int al);
	float v2_RVNS(float incumbentmain2);
	float v3_RVNS(float incumbentmain3);
	float v3_RVNS_AUX(float incumbentmain3, int al);
	float v4_RVNS(float incumbentmain4);
	float v5_RVNS(float incumbentmain5);
}rvns;

class VND : public GVNS
{
public:

	int q_vnd1 = 0;
	int q_vnd2 = 0;
	int q_vnd3 = 0;
	int q_vnd4 = 0;

	float VND_intensificacao(int ch, int gd, float sol_incumbent);
	float v1_VND(int ch1, float incumbentv1); //mover para adjacente
	float v2_VND(int ch2, float incumbentv2); //mover para adjacente do adjacente

	float v3_VND(int GD1, float incumbentv3);

	float v4_VND(int GD2, float incumbentv4);

}vnd;

//------------------------------------------------------------

void ParametrosSistema::leitura_parametros()
{
	FILE* arquivo;

	if ((arquivo = fopen("dados135.txt", "r")) == NULL)
		return;

	for (int i = 1; i < linha_dados; i++)
	{
		fscanf(arquivo, "%d%d%f%f%f%f%d%d%f%d%d", &ps.noi[i], &ps.nof[i], &ps.lt_r[i], &ps.lt_x[i], &ps.s_nofr[i], &ps.s_nofq[i], &ps.candidato_aloc[i], &ps.estado_swt_vanila[i], &ps.dist_no[i], &ps.candidato_GD[i], &ps.candidato_CH[i]);
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

void ParametrosSistema::leituraCenarios()
{
	FILE* arqc = fopen("cenarios.txt", "r");

	if (arqc == NULL) { return; }
	else
	{
		for (int i = 1; i < num_c; i++)
		{
			fscanf(arqc, "%f%f%f%f%f%f", &ps.cenario_demanda[i], &ps.cenario_is[i], &ps.tempo_cenario[i], &ps.custokwh[i], &ps.custotaxa_CO2[i], &ps.custoENS[i]);
		}
	}

	fclose(arqc);
}

void ParametrosSistema::somatorio_potencia()
{
	//aqui se soma a potencia ativa total, complexa e ativa
	ps.total_ativa = 0.0;
	ps.total_complexa = 0.0;

	for (int i = 1; i < linha_dados; i++)
	{
		ps.total_ativa = ps.total_ativa + ps.s_nofr[i];
		ps.total_complexa = ps.total_complexa + ps.s_nof[i];
	}
}

float ParametrosSistema::potenciaalimentador(int barrasalimentador[linha_dados])
{
	float PW = 0;

	for (int i = 1; i < linha_dados; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (barrasalimentador[i] == ps.nof[j])
			{
				PW = PW + ps.s_nofr[j];
			}
		}
	}

	return PW;
}

void FluxoPotencia::camadas(int alimentador, int camadaalimentador[linha_dados][linha_dados])
{
	//zerar camada 
	bool add = false;

	for (int i = 1; i < linha_dados; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			camadaalimentador[i][j] = 0;
		}
	}

	//define a camada do alimentador 

	camadaalimentador[1][1] = alimentador;

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

					if (ps.noi[k] == camadaalimentador[i][j])
					{
						if (ps.estado_swt[k] == 1)
						{
							for (int m = 1; m < linha_dados; m++)
							{
								for (int p = 1; p < linha_dados; p++)
								{
									if (ps.nof[k] == camadaalimentador[m][p])
									{
										add = true; //ja tem o elemento na matriz
									}
								}
							}

							if (add == false)
							{
								camadaalimentador[x][y] = ps.nof[k];
								y++;
							}

						}
					}
				}
				//

				for (int k = 1; k < linha_dados; k++)
				{
					add = false;

					if (ps.nof[k] == camadaalimentador[i][j])
					{
						if (ps.estado_swt[k] == 1)
						{
							for (int m = 1; m < linha_dados; m++)
							{
								for (int p = 1; p < linha_dados; p++)
								{
									if (ps.noi[k] == camadaalimentador[m][p])
									{
										add = true; //ja tem o elemento na matriz
									}
								}
							}

							if (add == false)
							{
								camadaalimentador[x][y] = ps.noi[k];
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

void FluxoPotencia::backward_sweep(int alimentador, int almt)
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
				if (fxp.camadaAL[almt][i][j] == ps.nof[k])
				{
					corrente_pu[k] = conj(ps.pu_s_nof[k] / tensao_pu[k]);
				}
			}
		}
	}

	// somatorio das correntes nos ramos
	for (int i = linha_dados - 1; i > 2; i--) //o i aqui vai ate 1 pq a linha 1 é a do alimentador e i-1 sera a linha 1
	{
		for (int k = linha_dados - 1; k > 0; k--)
		{
			if (fxp.camadaAL[almt][i][k] != 0)
			{
				for (int r = 1; r < linha_dados; r++)
				{
					if (fxp.camadaAL[almt][i - 1][r] == 0) { continue; }

					for (int j = 1; j < linha_dados; j++)
					{
						if (fxp.camadaAL[almt][i][k] == ps.nof[j] && fxp.camadaAL[almt][i - 1][r] == ps.noi[j])
						{
							posf = 0;
							posi = 0;

							for (int o = 1; o < linha_dados; o++)
							{
								if (ps.nof[o] == fxp.camadaAL[almt][i - 1][r] && ps.estado_swt[o] == 1)
								{
									if (ps.candidato_aloc[o] == 1 || ps.noi[o] == alimentador)
									{
										posf = o;
										break;
									}
								}
							}

							for (int o = 1; o < linha_dados; o++)
							{
								if (ps.nof[o] == fxp.camadaAL[almt][i][k] && ps.estado_swt[o] == 1)
								{
									if (ps.candidato_aloc[o] == 1 || ps.noi[o] == alimentador)
									{
										posi = o;
										break;
									}
								}
							}

							fxp.corrente_pu[posf] = fxp.corrente_pu[posf] + fxp.corrente_pu[posi];
						}
						else if (fxp.camadaAL[almt][i][k] == ps.noi[j] && fxp.camadaAL[almt][i - 1][r] == ps.nof[j])
						{
							posf = 0;
							posi = 0;

							for (int o = 1; o < linha_dados; o++)
							{
								if (ps.nof[o] == fxp.camadaAL[almt][i - 1][r] && ps.estado_swt[o] == 1)
								{
									if (ps.candidato_aloc[o] == 1 || ps.noi[o] == alimentador)
									{
										posf = o;
										break;
									}
								}
							}

							for (int o = 1; o < linha_dados; o++)
							{
								if (ps.nof[o] == fxp.camadaAL[almt][i][k] && ps.estado_swt[o] == 1)
								{
									if (ps.candidato_aloc[o] == 1 || ps.noi[o] == alimentador)
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

void FluxoPotencia::forward_sweep(int alimentador, int alt)
{

	complex <float> unit = fxp.tensao_inicial; //complexo unitario em pu para a tensao
	int vf = 0;
	int vi = 0;

	//atribuindo a tensao para o restante dos nós

	for (int i = 1; i < (linha_dados - 2); i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (fxp.camadaAL[alt][i][j] == 0) { continue; } //continua se for 0 para adiantar o processo

			for (int k = 1; k < linha_dados; k++)
			{
				if (fxp.camadaAL[alt][i + 1][k] == 0) { continue; }

				for (int t = 1; t < linha_dados; t++)
				{
					if (fxp.camadaAL[alt][i][j] == alimentador)
					{
						vf = 0;
						for (int r = 1; r < linha_dados; r++)
						{
							if (ps.noi[r] == alimentador)
							{
								vf = r;
								break;
							}
						}

						fxp.tensao_pu[vf] = unit - (fxp.corrente_pu[vf] * ps.pu_lt[vf]);
						break;
					}
					else if (fxp.camadaAL[alt][i][j] == ps.noi[t] && fxp.camadaAL[alt][i + 1][k] == ps.nof[t] && fxp.camadaAL[alt][i][j] != alimentador)
					{
						vf = 0;
						vi = 0;

						for (int r = 1; r < linha_dados; r++)
						{
							if (ps.nof[r] == fxp.camadaAL[alt][i][j] && ps.estado_swt[r] == 1)
							{
								vi = r;
								break;
							}
						}

						for (int r = 1; r < linha_dados; r++)
						{
							if (ps.nof[r] == fxp.camadaAL[alt][i + 1][k] && ps.estado_swt[r] == 1)
							{
								vf = r;
								break;
							}
						}

						fxp.tensao_pu[vf] = fxp.tensao_pu[vi] - (fxp.corrente_pu[vf] * ps.pu_lt[t]);

					}

					else if (fxp.camadaAL[alt][i][j] == ps.nof[t] && fxp.camadaAL[alt][i + 1][k] == ps.noi[t] && fxp.camadaAL[alt][i][j] != alimentador)
					{
						vf = 0;
						vi = 0;

						for (int r = 1; r < linha_dados; r++)
						{
							if (ps.nof[r] == fxp.camadaAL[alt][i][j] && ps.estado_swt[r] == 1)
							{
								vi = r;
								break;
							}
						}

						for (int r = 1; r < linha_dados; r++)
						{
							if (ps.nof[r] == fxp.camadaAL[alt][i + 1][k] && ps.estado_swt[r] == 1)
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

float FluxoPotencia::perdas_ativa(int barras[linha_dados])
{
	//perdas ativas no ramo

	bool passe = false;

	complex<float>Vr;
	complex<float>Ir;

	int aux = 0;

	float perdasW = 0.0;

	for (int i = 1; i < linha_dados; i++)
	{
		for (int ii = 1; ii < linha_dados; ii++)
		{
			if (ps.nof[i] == barras[ii])
			{
				passe = true;
			}
		}

		if (passe)
		{
			if (ps.noi[i] > 999)
			{
				Vr = (fxp.tensao_inicial - fxp.tensao_pu[i]) * vref;
				Ir = Vr / (ps.pu_lt[i] * zref);
				perdasW = perdasW + ((ps.pu_lt[i].real() * zref) * abs(Ir));
			}
			else
			{
				for (int k = 1; k < linha_dados; k++)
				{
					if (ps.noi[i] == ps.nof[k])
					{
						aux = k;
						break;
					}
				}

				Vr = (fxp.tensao_pu[aux] - fxp.tensao_pu[i]) * vref;
				Ir = Vr / (ps.pu_lt[i] * zref);

				perdasW = perdasW + ((ps.pu_lt[i].real() * zref) * abs(Ir));
			}
		}
		
	}

	perdasW = perdasW / 1000; //colocar em kW
	
	return perdasW;
}

void FluxoPotencia::fluxo_potencia(bool flagCAMADA) //alterar conforme o numero de alimentadores, modificando quantas vezes cada funcao eh chamada
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
	//definir as camadas de fxp.camadas de cada alimentador

	//******FLAG - volta ao vanila

	if (flagCAMADA)
	{
		//estado das chaves igual o vanila
		for (int k = 1; k < linha_dados; k++)
		{
			ps.estado_swt[k] = ps.estado_swt_vanila[k];
		}

		//camadas vanila
		for (int i = 1; i < num_AL; i++)
		{
			camadas(alimentadores[i], fxp.camadaAL[i]);
		}

		return;
	}

	//camadas
	for (int i = 1; i < num_AL; i++)
	{
		camadas(alimentadores[i], fxp.camadaAL[i]);
	}

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
		for (int i = 1; i < num_AL; i++)
		{
			backward_sweep(alimentadores[i], i);
		}

		//2 passo: FORWARD

		//tensoes nodais
		for (int i = 1; i < num_AL; i++)
		{
			forward_sweep(alimentadores[i], i);
		}

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

void FluxoPotencia::camadas_simples(int referencia, int camadailha[linha_dados][linha_dados])
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

void FluxoPotencia::backward_sweep_simples(int referencia)
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

void FluxoPotencia::forward_sweep_simples(int referencia)
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

void FluxoPotencia::fluxo_potencia_simples(int referencia) //alterar conforme o numero de alimentadores, modificando quantas vezes cada funcao eh chamada
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
		backward_sweep_simples(referencia);

		//2 passo: FORWARD

		//tensoes nodais
		forward_sweep_simples(referencia);

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
	float potGDs = 0.0;

	//nesta etapa apenas retorna um valor verdadeiro ou falso para simplificar codigos adiante, mas nao passa do calculo do fluxo de potencia
	fluxo_potencia_simples(barrareferencia);

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

	//checando potencia
	if (barrareferencia >= 1000)
	{
		resultado = true;
	}
	else
	{	
		potGDs = 0.0;
		somaPotencia = 0.0;

		//sera somente o GD, ai tem que conferir
		for (int i = 1; i < linha_dados; i++)
		{
			for (int j = 1; j < linha_dados; j++)
			{
				if (fxp.camadaREF[i][j] != 0)
				{
					for (int k = 1; k < linha_dados; k++)
					{
						if (ps.nof[k] == fxp.camadaREF[i][j])
						{
							somaPotencia += ps.s_nofr[k];
						}
					}
				}
			}
		}

		//somar potencia do gd
		for (int i = 1; i < linha_dados; i++)
		{
			for (int j = 1; j < linha_dados; j++)
			{
				if (fxp.camadaREF[i][j] != 0)
				{
					for (int k = 1; k < linha_dados; k++)
					{
						if (ps.nof[k] == fxp.camadaREF[i][j] && agd.GDbarra[k] != 0)
						{
							potGDs += ps.s_nofr[k];
						}
					}
				}
			}
		}

		//conferir
		if (potGDs > somaPotencia) { resultado = true; }
		else { resultado = false; }
	}

	return (resultado);
}

void AlocacaoChaves::adjacentes(int posicao[linha_dados], int adj[linha_dados][linha_dados], int alimentador)
{
	int contline = 0; //contador da linha
	int contadorch = 0;
	int aux = 0; //auxiliar nas barras

	contline = 0;

	for (int i = 1; i < linha_dados; i++)
	{
		contadorch = 1;

		for (int j = 1; j < linha_dados; j++)
		{
			if (posicao[i] == j)
			{
				contline++;
				adj[contline][contadorch] = ps.nof[j];

				aux = 1;

				while (aux != linha_dados)
				{
					//localizando adjacentes
					for (int k = 1; k < linha_dados; k++)
					{
						if (ps.noi[k] == adj[contline][aux] && ps.candidato_aloc[k] == 1)
						{
							contadorch++;
							adj[contline][contadorch] = ps.nof[k];
						}
					}

					aux++;
				}
			}
		}

	}


	//agora pega a secao do alimentador

	contadorch = 1; //coloca o contador na primeira posicao

	for (int j = 1; j < linha_dados; j++)
	{
		if (alimentador == ps.noi[j])
		{
			contline++;
			adj[contline][contadorch] = ps.nof[j];

			aux = 1;

			while (aux != linha_dados)
			{
				//localizando adjacentes
				for (int k = 1; k < linha_dados; k++)
				{
					if (ps.noi[k] == adj[contline][aux] && ps.candidato_aloc[k] == 1)
					{
						contadorch++;
						adj[contline][contadorch] = ps.nof[k];
					}
				}

				aux++;
			}
		}
	}
}

void AlocacaoChaves::secoes_alimentador()
{
	int cont_ch[num_AL][linha_dados]; //contador de barras das chaves
	int cont = 0; //contador
	int pos = 0;

	//zerar matrizes dos adjacentes
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			for (int k = 1; k < linha_dados; k++)
			{
				ac.secoes_chaves[i][j][k] = 0;
				ac.adjacente_chaves[i][j][k] = 0;
			}
		}
	}

	//encontrando todos os adjacentes
	for (int i = 1; i < num_AL; i++)
	{
		adjacentes(ac.posicaochaves[i], ac.adjacente_chaves[i], alimentadores[i]);
	}

	//zera contadores 
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			cont_ch[i][j] = 0;
		}
	}

	//

	cont = 0;

	//determina qual das chaves tem mais barras adjacentes
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			for (int k = 1; k < linha_dados; k++)
			{
				if (ac.adjacente_chaves[i][j][k] != 0)
				{
					cont++;
				}
			}
			cont_ch[i][j] = cont;
			cont = 0;
		}
	}

	//coloca em ordem a matriz antes de zerar


	for (int i = 1; i < num_AL; i++)
	{
		int xx = 0;
		xx = 0;
	agn:

		//pegando a maior
		cont = 0;
		pos = 1;

		for (int j = 1; j < linha_dados; j++)
		{
			if (cont < cont_ch[i][j])
			{
				cont = cont_ch[i][j];
				pos = j;
			}
		}

		cont_ch[i][pos] = 0;
		xx++;

		if (cont != 0)
		{
			for (int j = 1; j < linha_dados; j++)
			{
				ac.secoes_chaves[i][xx][j] = ac.adjacente_chaves[i][pos][j];
			}
			goto agn;
		}

		//aproveitar para colocar as barras adjacentes da chave em ordem

		//zerar para nao ter problemas futuros
		for (int j = 1; j < linha_dados; j++)
		{
			for (int k = 1; k < linha_dados; k++)
			{
				ac.adjacente_chaves[i][j][k] = 0;
			}

		}

		//ordenando...
		for (int j = 1; j < linha_dados; j++)
		{
			for (int k = 1; k < linha_dados; k++)
			{
				ac.adjacente_chaves[i][j][k] = ac.secoes_chaves[i][j][k];
			}

		}

		//agora sim, zerar os repetidos da array ac.adjacente_chaves[][][]

		for (int j = 1; j < linha_dados; j++)
		{
			for (int k = 1; k < linha_dados; k++)
			{
				//
				if (ac.secoes_chaves[i][j][k] == 0)
				{
					continue;
				}

				for (int y = 1; y < linha_dados; y++)
				{
					for (int p = 1; p < linha_dados; p++)
					{
						if (ac.secoes_chaves[i][y][p] == 0)
						{
							continue;
						}
						if (ac.secoes_chaves[i][j][k] == ac.secoes_chaves[i][y][p] && y != j)
						{
							ac.secoes_chaves[i][j][k] = 0;
						}
					}
				}
			}
		}
	}
}

float AlocacaoChaves::energia_suprida(int AL, vector<int>barras_AL)
{
	bool analise = false;
	bool conta_pot = false;
	float energia_sup = 0.0;
	complex <float> capacidadeSE;

	// ANALISANDO FLUXO DE POTENCIA:

	//faz o fluxo de potencia para a situação do remanejamento
	fxp.fluxo_potencia(false);

	analise = true; //verdadeiro se atende as condiçoes, falso caso contrario -> assume-se inicialmente

	// 1. Níveis de tensao e corrente
	for (int y = 1; y < linha_dados; y++)
	{
		//para a tensao:
		if (abs(fxp.tensao_pu[y]) < estado_restaurativo_pu && abs(fxp.tensao_pu[y]) != 0)
		{
			analise = false;
			break;
		}
	}

	// 2. A potencia alimentada por cada alimentador deve estar dentro da capacidade do alimentador
	for (int i = 1; i < num_AL; i++)
	{
		capacidadeSE.real(0.0);
		capacidadeSE.imag(0.0);

		for (int j = 1; j < linha_dados; j++)
		{
			for (int k = 1; k < linha_dados; k++)
			{
				for (int t = 1; t < linha_dados; t++)
				{
					if (fxp.camadaAL[i][j][k] == ps.nof[t] && fxp.camadaAL[i][j][k] != 0 && ps.estado_swt[t] == 1)
					{
						capacidadeSE += ps.s_nof[t];
					}
				}
			}
		}

		if (abs(capacidadeSE) > capSE)
		{
			analise = false;
			break;
		}
	}


	//feita a analise do fluxo de putencia:
	if (analise == true)
	{
		energia_sup = 0.0;

		//ver a energia suprida neste caso
		for (int i = 0; i < barras_AL.size(); i++)
		{
			conta_pot = false;

			for (int u = 1; u < num_AL; u++)
			{
				if (AL == u) { continue; }
				else
				{
					for (int k = 2; k < linha_dados; k++) //começa no 2 para tirar o al da analise
					{
						for (int j = 1; j < linha_dados; j++)
						{
							if (fxp.camadaAL[u][k][j] != 0)
							{
								if (fxp.camadaAL[u][k][j] == barras_AL[i]) { conta_pot = true; }
							}
						}
					}
				}
			}

			if (conta_pot == true)
			{
				for (int k = 1; k < linha_dados; k++)
				{
					if (barras_AL[i] == ps.nof[k])
					{
						energia_sup = energia_sup + ps.s_nofr[k];
					}
				}
			}
		}
	}
	else
	{
		energia_sup = 0.0;
	}


	//retornar valor a variavel
	return energia_sup;
}

float FuncaoObjetivo::FO(float potenciaAL, float comprimento, float ens)
{
	float resultado = 0.0;

	if (ens != 0)
	{
		resultado = taxa_falhas * comprimento * ((potenciaAL * tempo_isolacao) + (tempo_falha * ens));
	}

	return resultado;
}

void AlocacaoChaves::chaves_anteriores()
{
	//salva as chaves
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			ac.antchi[i][j] = ac.chi[i][j];
			ac.antchf[i][j] = ac.chf[i][j];
			ac.antpos[i][j] = ac.posicaochaves[i][j];
		}
	}
}

void AlocacaoChaves::volta_chaves_anteriores()
{
	//volta as chaves
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			ac.chi[i][j] = ac.antchi[i][j];
			ac.chf[i][j] = ac.antchf[i][j];
			ac.posicaochaves[i][j] = ac.antpos[i][j];
		}
	}
}

void AlocacaoGD::gd_anteriores()
{
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			agd.antGD[i][j] = agd.posicaoGD[i][j];
		}
	}

	for (int i = 1; i < linha_dados; i++)
	{
		agd.ant_quantGD[i] = agd.quantGD[i];
	}
}

void AlocacaoGD::volta_gd_anteriores()
{
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			agd.posicaoGD[i][j] = agd.antGD[i][j];
		}
	}

	for (int i = 1; i < linha_dados; i++)
	{
		agd.quantGD[i] = agd.ant_quantGD[i];
	}
}

void AlocacaoGD::atualizaPosGD()
{
	//funcao para atualizar a posicao dos GDs

	//zerar mx de GDs
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			agd.posicaoGD[i][j] = 0;
		}
	}

	//atribuir as posicoes do gd
	int index_linha = 0;
	int index_coluna = 0;

	for (int k = 1; k < linha_dados; k++)
	{
		if (ps.noi[k] > 999)
		{
			index_linha++;
			index_coluna = 1;
		}

		if (agd.quantGD[k] != 0)
		{
			agd.posicaoGD[index_linha][index_coluna] = k;
			index_coluna++;
		}
	}
}

float AlocacaoGD::PotW(float IS)
{
	float pv = 0.0;

	pv = (5525 * IS) + (2925 * pow(IS, 2)); //equacao da potencia para cada cenario de um modulo pv

	pv = pv / 1000; //converte para kW

	pv = 25 * pv; //junta 25 modulos para dar o custo

	return pv;
}

void AlocacaoGD::ajustePOT(complex<float>S[linha_dados], float P[linha_dados], float P_gd[linha_dados], float demanda, int posGD[linha_dados])
{
	for (int k = 1; k < linha_dados; k++)
	{
		if (posGD[k] != 0)
		{
			S[posGD[k]] = S[posGD[k]] * demanda;
			P[posGD[k]] = P[posGD[k]] - P_gd[posGD[k]];
		}
	}
}

void AlocacaoGD::potenciaGD(float ISpv, float PGD[linha_dados], int numGDbarra[linha_dados])
{
	//zerar vetor potencia na barra
	for (int i = 1; i < linha_dados; i++)
	{
		PGD[i] = 0;
	}

	//calcular potencia na barra do gd
	for (int i = 1; i < linha_dados; i++)
	{

		PGD[i] = numGDbarra[i] * PotW(ISpv); //potencia na barra = numero blocos de gd * potencia de cada bloco
		
	}
}

void AlocacaoGD::distribuicao_potencia(int secao[linha_dados])
{
	//distribui potencia se na barra for negativa para barras na mesma secao da chave
	float potenciaDistribuida = 0.0;
	bool disPOT = false;
	for (int i = 1; i < linha_dados; i++)
	{
		if (secao[i] == 0) continue;

		for (int j = 1; j < linha_dados; j++)
		{
			if (secao[i] == ps.nof[j] && ps.s_nofr[j] < 0)
			{
				potenciaDistribuida += ps.s_nofr[j];
				ps.s_nofr[j] = 0;
				disPOT = true;
			}
		}
	}

	//distribuir potencia nas barras da secao
	if (!disPOT)return;

	bool condicaoDistribuir = true;
	for (int i = 1; i < linha_dados; i++)
	{
		if (secao[i] == 0) continue;
		for (int j = 1; j < linha_dados; j++)
		{
			if (secao[i] == ps.nof[j] && ps.s_nofr[j] > 0 && condicaoDistribuir)
			{
				ps.s_nofr[j] += potenciaDistribuida;
				
				//condicional para saber se continua ou não distribuindo a potencia
				if (ps.s_nofr[j] < 0)
				{
					potenciaDistribuida = ps.s_nofr[j];
					ps.s_nofr[j] = 0;
				}
				else
				{
					condicaoDistribuir = false;
				}
			}
		}
	}
}

bool AlocacaoGD::opILHA(int secoesCM[linha_dados][linha_dados], int adjCM[linha_dados][linha_dados], int posGD[linha_dados], int AL, int secao)
{
	bool operacao = false;

	for (int i = 1; i < linha_dados; i++)
	{
		if (i == secao) { continue; }

		for (int k = 1; k < linha_dados; k++)
		{
			if (secoesCM[i][k] == AL) //alimentador
			{
				operacao = fxp.condicaoTensaoFluxoPotencia(AL);
			}

			for (int j = 1; j < linha_dados; j++)
			{
				if (ps.nof[posGD[j]] == secoesCM[i][k] && posGD[j] != 0) //na secao tem GD
				{
					operacao = fxp.condicaoTensaoFluxoPotencia(ps.nof[posGD[j]]);
				}
				
			}
		}
	}

	return operacao;
}

float FuncaoObjetivo::calculo_funcao_objetivo(bool all_al, int p_al)
{
	float comprimento_secao = 0.0;
	float potencia_W = 0.0;
	float perdas = 0.0;
	float potencia_isolacao = 0.0;
	float ens = 0.0;
	float ENSt = 0.0;


	int intervaloA = 0;
	int intervaloB = 0;
	int contcondicao = 0;

	vector <int> secao = {};
	vector <int> posicao = {};
	vector <int> barras = {};
	vector <vector<int>> analise_remanejamento = {};
	vector <vector<int>> remanej_cargas = {};
	vector <vector<int>> remanej_cargas2 = {};

	posicao.clear();
	secao.clear();
	analise_remanejamento.clear();
	remanej_cargas.clear();

	//deve-se analisar todas as secoes para os valores da funcao obj

	ac.secoes_alimentador();
	ps.leitura_parametros();
	fxp.fluxo_potencia(false);

	//analisando condicao
	if (all_al)
	{
		intervaloA = 1;
		intervaloB = num_AL;
	}
	else
	{
		intervaloA = p_al;
		intervaloB = p_al + 1;
	}

	//secao j do alimentador w
	for (int w = intervaloA; w < intervaloB; w++)
	{
		//barras do alimentador w
		barras.clear();
		for (int q = 1; q < linha_dados; q++)
		{
			if (ac.adjacente_chaves[w][1][q] != 0)
			{
				barras.push_back(ac.adjacente_chaves[w][1][q]);
			}
		}

		for (int i2 = 1; i2 < num_c; i2++)
		{		
			perdas = 0.0;
			ENSt = 0.0;

			//potencia em cada modulo PV
			agd.potenciaGD(ps.cenario_is[i2], agd.GDbarra, agd.quantGD); //ajusta potencia do GD

			//ajustando potencia nas barras
			agd.ajustePOT(ps.pu_s_nof, ps.s_nofr, agd.GDbarra, ps.cenario_demanda[i2], agd.posicaoGD[w]); //ajusta a potencia para fazer o fluxo de potencia

			//distribuir potencia dos gds na secao caso necessário
			for (int numSecao = 1; numSecao <= ac.numch_AL[w] + 1; numSecao++)
			{
				agd.distribuicao_potencia(ac.secoes_chaves[w][numSecao]);
			}

			//realizar somatorios
			ps.somatorio_potencia();
			ps.potencia_al = ps.potenciaalimentador(ac.adjacente_chaves[w][1]); //toda a potencia do alimentador

			//calculo de perdas no alimentador para este cenario
			perdas = fxp.perdas_ativa(ac.adjacente_chaves[w][1]); //perdas no alimentador

			//secao j do alimetador w - ***CALCULO DA ENS ***
			for (int j = 1; j < linha_dados; j++)
			{
				//ver se vale a pena fazer o laço
				contcondicao = 0;

				for (int k = 0; k < linha_dados; k++)
				{
					if (ac.secoes_chaves[w][j][k] != 0)
					{
						contcondicao++;
					}
				}

				if (contcondicao == 0) { continue; }

				////////////////////////////
				//começo da secao

				comprimento_secao = 0.0;
				potencia_W = 0.0;
				potencia_isolacao = 0.0;

				// k = barras da seção j
				
				//Inicio da analise do remanejamento
				// ************************************
				
				//analise comprimento e potencia nao suprida
				for (int k = 1; k < linha_dados; k++)
				{
					//comprimento
					for (int y = 1; y < linha_dados; y++)
					{
						if (ac.secoes_chaves[w][j][k] == ps.nof[y])
						{
							comprimento_secao = comprimento_secao + ps.dist_no[y];
						}
					}
				}

				// 1) primeiro deve-se pegar toda a area do alimentador e deliga-la
				for (int k = 1; k < linha_dados; k++)
				{
					for (int y = 1; y < linha_dados; y++)
					{
						if (ac.adjacente_chaves[w][1][k] == ps.nof[y])
						{
							potencia_isolacao = potencia_isolacao + ps.s_nofr[y];
						}
					}
				}

				// 2) agora deve-se fazer o devido chaveamento ou operacao em ilha 

				// 2a) isolando secao j abrindo a chave
				for (int k = 1; k < linha_dados; k++)
				{
					for (int y = 1; y < linha_dados; y++)
					{
						if (ac.secoes_chaves[w][j][k] == 0 || ac.posicaochaves[w][y] == 0) { continue; }
						else if (ac.secoes_chaves[w][j][k] == ps.nof[ac.posicaochaves[w][y]] || ac.secoes_chaves[w][j][k] == ps.noi[ac.posicaochaves[w][y]])
						{
							ps.estado_swt[ac.posicaochaves[w][y]] = 0;
						}
					}
				}

				if (j == 1) //desligar o disjuntor da subestacao
				{
					for (int k = 1; k < linha_dados; k++)
					{
						if (ps.noi[k] == alimentadores[w])
						{
							ps.estado_swt[k] = 0;
						}
					}
				}

				// 2b) cenario da falta
				//zerar falha na camada
				for (int k = 1; k < linha_dados; k++)
				{
					for (int y = 1; y < linha_dados; y++)
					{
						for (int t = 1; t < linha_dados; t++)
						{
							if (fxp.camadaAL[w][k][y] == ac.secoes_chaves[w][j][t])
							{
								fxp.camadaAL[w][k][y] = 0;
							}
						}
					}
				}

				bool inicio;
				int cont_nao0;

				inicio = false;

				for (int k = 1; k < linha_dados; k++)
				{
					cont_nao0 = 0;

					for (int y = 1; y < linha_dados; y++)
					{
						if (fxp.camadaAL[w][k][y] != 0)
						{
							secao.push_back(fxp.camadaAL[w][k][y]);
							cont_nao0++;
						}
					}

					if (cont_nao0 == 0)
					{
						secao.clear();
						inicio = true;
					}
					else if (inicio == true && secao.size() != 0)
					{
						for (int m = 1; m < linha_dados; m++)
						{
							for (int n = 1; n < linha_dados; n++)
							{
								for (int t = 0; t < secao.size(); t++)
								{
									if (secao[t] == ac.secoes_chaves[w][m][n])
									{
										posicao.push_back(m);
									}
								}
							}
						}

						secao.clear();
					}
					else
					{
						secao.clear();
					}
				}

				//eliminar elementos iguais no vetor
				for (int k = 0; k < posicao.size(); k++)
				{
					for (int t = 0; t < posicao.size(); t++)
					{
						if (t != k && posicao[k] == posicao[t])
						{
							posicao[t] = 0;
						}
					}
				}

				//analisando o remanejamento:
				secao.clear();

				for (int k = 0; k < posicao.size(); k++)
				{
					if (posicao[k] == 0) { continue; }

					for (int t = 1; t < linha_dados; t++)
					{
						if (ac.adjacente_chaves[w][posicao[k]][t] != 0)
						{
							secao.push_back(ac.adjacente_chaves[w][posicao[k]][t]);
						}
					}

					//verificar se esta contido na camada
					bool contbar;

					contbar = false;

					for (int g = 0; g < analise_remanejamento.size(); g++)
					{
						for (int h = 0; h < analise_remanejamento[g].size(); h++)
						{
							for (int t = 0; t < secao.size(); t++)
							{
								if (analise_remanejamento[g][h] == secao[t]) { contbar = true; }
							}
						}
					}

					if (contbar == false)
					{
						analise_remanejamento.push_back(secao);
						secao.clear();
					}
					else
					{
						secao.clear();
					}


				}

				//pegando posições
				posicao.clear();

				for (int k = 0; k < analise_remanejamento.size(); k++)
				{
					for (int t = 0; t < analise_remanejamento[k].size(); t++)
					{
						for (int y = 1; y < linha_dados; y++)
						{
							if (ps.noi[y] == analise_remanejamento[k][t] && ps.candidato_aloc[y] == 0)
							{
								posicao.push_back(y);
							}
							else if (ps.nof[y] == analise_remanejamento[k][t] && ps.candidato_aloc[y] == 0)
							{
								posicao.push_back(y);
							}
						}
					}

					if (!posicao.empty())
					{
						remanej_cargas.push_back(posicao);
						posicao.clear();
					}
				}

				//Fim da analise do remanejamento
				//***************************************

				//Inicio dos calculos para o cenário

				//3) Calculo da ENS pelo sistema caso ocorra falha na seção j do alimentador i para cada um dos cenarios possiveis em um ano

				bool operacaoILHA = false;
				//3.1) Operação em Ilha
				
				if (ps.cenario_is[i2] != 0)
				{
					//fechando chave da secao
					for (int k = 1; k < linha_dados; k++)
					{
						for (int y = 1; y < linha_dados; y++)
						{
							if (ac.secoes_chaves[w][j][k] == 0 || ac.posicaochaves[w][y] == 0) { continue; }
							else if (ac.secoes_chaves[w][j][k] == ps.nof[ac.posicaochaves[w][y]] || ac.secoes_chaves[w][j][k] == ps.noi[ac.posicaochaves[w][y]])
							{
								ps.estado_swt[ac.posicaochaves[w][y]] = 0;
							}
						}
					}
					operacaoILHA = agd.opILHA(ac.secoes_chaves[w], ac.adjacente_chaves[w], agd.posicaoGD[w], alimentadores[w], j);

					//voltar ao estado vanila - não dar B.O
				}
				
				//REMANEJAMENTO *********
				//3.2) A operação em ilha nao acontece, sendo necessário fazer o remanejamento de cargas
				if (!operacaoILHA)
				{
					if (remanej_cargas.size() != 0)
					{
						//cout << "remanej ok" << endl;
						ens = ps.potencia_al;

						//somente uma opcao para remanejamento de cada vez
						for (int a = 0; a < remanej_cargas.size(); a++)
						{
							potencia_W = 0.0;

							for (int b = 0; b < remanej_cargas[a].size(); b++)
							{
								//fechando chave da secao
								for (int k = 1; k < linha_dados; k++)
								{
									for (int y = 1; y < linha_dados; y++)
									{
										if (ac.secoes_chaves[w][j][k] == 0 || ac.posicaochaves[w][y] == 0) { continue; }
										else if (ac.secoes_chaves[w][j][k] == ps.nof[ac.posicaochaves[w][y]] || ac.secoes_chaves[w][j][k] == ps.noi[ac.posicaochaves[w][y]])
										{
											ps.estado_swt[ac.posicaochaves[w][y]] = 0;
										}
									}
								}

								//abrindo chave
								ps.estado_swt[remanej_cargas[a][b]] = 1;

								potencia_W = ac.energia_suprida(w, barras);

								ps.estado_swt[remanej_cargas[a][b]] = 0;

								if (potencia_W != 0) { break; }

							}

							ens = ens - potencia_W;

						}

						if (ens == ps.potencia_al)
						{
							//nao tem como fazer manobra, a ENS será os adjacentes da chave
							ens = 0.0;

							for (int y = 1; y < linha_dados; y++)
							{
								for (int h = 1; h < linha_dados; h++)
								{
									if (ps.nof[h] == ac.adjacente_chaves[w][j][y])
									{
										ens = ens + ps.s_nofr[h];
									}
								}
							}

							//voltar ao estado vanila - não dar B.O
						}
					}
					//sem manobra
					else
					{
						//nao tem como fazer manobra, a ENS será os adjacentes da chave
						//cout << "sem remanje" << endl;
						ens = 0.0;

						for (int y = 1; y < linha_dados; y++)
						{
							for (int h = 1; h < linha_dados; h++)
							{
								if (ps.nof[h] == ac.adjacente_chaves[w][j][y])
								{
									ens = ens + ps.s_nofr[h];
								}
							}
						}

						if (ens < 0)
						{ 
							ens = 0; 

						} //funciona ilhado
					}
				}
				//**COM OPERAÇÃO EM ILHA**
				else
				{
					//a ens é a soma das potencias da secao somente
					//cout << "opILHA ok" << endl;
					ens = 0.0;

					for (int y = 1; y < linha_dados; y++)
					{
						for (int h = 1; h < linha_dados; h++)
						{
							if (ps.nof[h] == ac.secoes_chaves[w][j][y])
							{
								ens = ens + ps.s_nofr[h];
							}
						}
					}

					if (ens < 0)
					{
						ens = 0;
					}

					//voltar ao estado vanila - não dar B.O
					ps.leitura_parametros();
					fxp.fluxo_potencia(true);
					agd.ajustePOT(ps.pu_s_nof, ps.s_nofr, agd.GDbarra, ps.cenario_demanda[i2], agd.posicaoGD[w]);
				}

				//4 custos do cenario

				//aviso de erro
				if (ens < 0) { 
					cout << "report error - alimentador: " << w << endl; }

				//calculo da ENS pelo método da IC 2020
				ENSt = ENSt + FO(potencia_isolacao, comprimento_secao, ens);
				ens = 0.0;

				//zera vetores
				analise_remanejamento.clear();
				remanej_cargas.clear();
				secao.clear();
				posicao.clear();
			}

			//fim na secao - CALCULO DOS CUSTOS -
			fo.custoENS += ENSt * ps.tempo_cenario[i2] * ps.custoENS[i2]; //custo energia energia nao suprida
			fo.custoPe += perdas * ps.tempo_cenario[i2] * ps.custokwh[i2]; //custo perdas
			fo.custoP += ps.potencia_al * ps.tempo_cenario[i2] * ps.custokwh[i2]; //custo potencia comprada pelo alimentador
			fo.custoEmss += fo.eCO2 * (ps.potencia_al + perdas) * ps.tempo_cenario[i2] * ps.custotaxa_CO2[i2]; //custo emissao de poluentes

		}

		//Fim dos cenarios - CALCULO DOS INVESTIMENTOS -
		fo.investimentoCM = ac.numch_AL[w] * custoSW; //investimento das chaves de manobra
		fo.investimentoGD = agd.numgd_AL[w] * custoGD; //investimento para colocar o GD no sistema

		//*** CALCULO DA FUNCAO OBJETIVO TOTAL DO ALIMENTADOR ***
		fo_al[w] = fo.investimentoCM + fo.investimentoGD + fo.custoP + fo.custoEmss + fo.custoPe + fo.custoENS; //valor da FO para alimentador

		//zerar variaveis para o novo alimentador
		fo.investimentoCM = 0.0;
		fo.investimentoGD = 0.0;
		fo.custoENS = 0.0;
		fo.custoPe = 0.0;
		fo.custoP = 0.0;
		fo.custoEmss = 0.0;

		barras.clear();
	}

	float valorFO = 0.0;
	for (int i = 1; i < num_AL; i++)
	{
		valorFO = valorFO + fo_al[i]; //soma de todos os alimentadores
	}

	return valorFO;
}

int GVNS::localizaAL(int posicao)
{
	//localiza o alimentador
	//variaveis locais
	int posicao_alimentador = 0;

	for (int i = 1; i < linha_dados; i++)
	{
		if (ps.noi[i] > 999)
		{
			posicao_alimentador++;
		}
		if (posicao == i)
		{
			break;
		}
	}

	return posicao_alimentador;
}

void GVNS::sorteiochaves(int numch, int camada[linha_dados][linha_dados], int posicao_camada[linha_dados], int alimentador)
{
	int barras_camada[linha_dados];
	int aux = 0;
	int sort = 0;
	bool atribuir = false; //variavel auxiliar
	bool igual = false;

	//zera barra_camada
	for (int i = 1; i < linha_dados; i++)
	{
		barras_camada[i] = 0;
	}

	//atribui as barras do alimentador
	aux = 1;

	for (int i = 1; i < linha_dados; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (camada[i][j] != 0)
			{
				barras_camada[aux] = camada[i][j];
				aux++;
			}
		}
	}

	//sorteia
sorteio:
	for (int i = 1; i <= numch; i++)
	{
	rand_dnv:

		atribuir = false;

		sort = rand() % aux + 1;

		sort = barras_camada[sort];

		if (sort != 0)
		{
			//localizar posicao
			for (int j = 1; j < linha_dados; j++)
			{
				if (sort == ps.nof[j] && ps.candidato_CH[j] == 1 && ps.noi[j] != alimentador)
				{
					posicao_camada[i] = j;
					atribuir = true;
				}
			}

			if (atribuir == false)
			{
				goto rand_dnv;
			}
		}
		else
		{
			goto rand_dnv;
		}
	}

	//analisa se as tres sao diferentes
	igual = false;
	for (int i = 1; i < linha_dados; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (posicao_camada[i] == posicao_camada[j] && posicao_camada[i] != 0 && i != j)
			{
				igual = true;
			}
		}
	}

	if (igual == true)
	{
		goto sorteio;
	}
}

void GVNS::sorteioGDs(int numgd, int camada[linha_dados][linha_dados], int posicao_camada[linha_dados], int alimentador, int qntGD[linha_dados])
{
	
	int barras_camada[linha_dados];
	int aux = 0;
	int sort = 0;
	bool atribuir = false; //variavel auxiliar
	bool igual = false;

	//zera barra_camada
	for (int i = 1; i < linha_dados; i++)
	{
		barras_camada[i] = 0;
	}

	//atribui as barras do alimentador
	aux = 1; //??????????? mas funciona

	for (int i = 1; i < linha_dados; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (camada[i][j] != 0)
			{
				barras_camada[aux] = camada[i][j];
				aux++;
			}
		}
	}

	//sorteia
	int i = 1;

	while (i <= numgd)
	{
		sort = rand() % aux + 1;

		sort = barras_camada[sort];

		if (sort != 0)
		{
			//localizar posicao
			for (int j = 1; j < linha_dados; j++)
			{
				if (sort == ps.nof[j] && ps.candidato_GD[j] == 1 && ps.noi[j] != alimentador)
				{
					if (qntGD[j] < maxGD)
					{
						posicao_camada[i] = j;
						qntGD[j] = qntGD[j] + 1;
						i++;
					}
				}
			}
		}
	}
}

void GVNS::primeiraaloc() //alterar conforme o numero de alimentadores, modificando quantas vezes cada funcao eh chamada
{
	for (int i = 1; i < num_AL; i++)
	{
		sorteiochaves(ac.numch_AL[i], fxp.camadaAL[i], ac.posicaochaves[i], alimentadores[i]);
		sorteioGDs(agd.numgd_AL[i], fxp.camadaAL[i], agd.posicaoGD[i], alimentadores[i], agd.quantGD);
	}
}

float VND::v1_VND(int ch1, float incumbentv1)
{
	//localizando chave
	int cont = 0;
	int al = 0;
	int pch = 0;
	int pos_ch = 0;

	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (ac.posicaochaves[i][j] != 0)
			{
				//incrementa contador das chaves
				cont++;
			}
			if (cont == ch1)
			{
				//encontrou a chave: pegar dados
				al = i;
				pch = j;
				pos_ch = ac.posicaochaves[i][j];
				cont++; //para nao voltar nesse loop
			}
		}
	}

	//possibilidades dos vizinhos mais proximos a candidato de alocação
	vector<int>adjch;
	for (int i = pos_ch + 1; i < linha_dados; i++)
	{
		if (ps.noi[i] > 999) //protecao
		{
			break;
		}
		if (ps.candidato_CH[i] == 1)
		{
			//ver se ja possui chave
			bool verifica = false;
			for (int k = 1; k < linha_dados; k++)
			{
				if (i == ac.posicaochaves[al][k] && ac.posicaochaves[al][k] != 0)
				{
					verifica = true;
				}
			}
			//adiciona ou nao
			if (!verifica)
			{
				adjch.push_back(i);
				break;
			}
		}
	}
	for (int i = pos_ch - 1; i > 1; i--)
	{
		if (ps.noi[i] > 999) //protecao
		{
			break;
		}
		if (ps.candidato_CH[i] == 1)
		{
			//ver se ja possui chave
			bool verifica = false;
			for (int k = 1; k < linha_dados; k++)
			{
				if (i == ac.posicaochaves[al][k] && ac.posicaochaves[al][k] != 0)
				{
					verifica = true;
				}
			}
			//adiciona ou nao
			if (!verifica)
			{
				adjch.push_back(i);
				break;
			}
		}
	}

	//começa a alocação
	float resultado = 0.0;
	for (int i = 0; i < adjch.size(); i++)
	{
		ac.chaves_anteriores(); //salva chaves anteriores

		//atualiza chaves
		ac.posicaochaves[al][pch] = adjch[i];
		ac.chi[al][pch] = ps.noi[adjch[i]];
		ac.chf[al][pch] = ps.nof[adjch[i]];

		//calcula
		resultado = fo.calculo_funcao_objetivo(false, al);

		if (resultado < incumbentv1)
		{
			return resultado;
		}
		else
		{
			ac.volta_chaves_anteriores();
			ac.secoes_alimentador();
		}
	}

	return incumbentv1;
}

float VND::v2_VND(int gd2, float incumbentv2)
{
	//mover GD para a posição vizinha

	//localiza GD
	int al = 0;
	int pgd = 0;
	int pos_gd = 0;
	int cont = 0;
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (agd.posicaoGD[i][j] != 0)
			{
				//incrementa contador das chaves
				cont++;
			}
			if (cont == gd2)
			{
				//encontrou a chave: pegar dados
				al = i;
				pgd = j;
				pos_gd = agd.posicaoGD[al][pgd];
				cont++; //para nao voltar nesse loop
			}
		}
	}

	//possibilidades dos vizinhos mais proximos a candidato de alocação
	vector<int>adjgd;
	for (int i = pos_gd + 1; i < linha_dados; i++)
	{
		if (ps.noi[i] > 999) //protecao
		{
			break;
		}
		if (ps.candidato_GD[i] == 1)
		{
			//ver se ja possui gd
			bool verifica = false;
			for (int k = 1; k < linha_dados; k++)
			{
				if (i == agd.posicaoGD[al][k] && agd.posicaoGD[al][k] != 0)
				{
					verifica = true;
				}
			}
			//adiciona ou nao
			if (!verifica)
			{
				adjgd.push_back(i);
				break;
			}
		}
	}
	for (int i = pos_gd - 1; i > 1; i--)
	{
		if (ps.noi[i] > 999) //protecao
		{
			break;
		}
		if (ps.candidato_GD[i] == 1)
		{
			//ver se ja possui chave
			bool verifica = false;
			for (int k = 1; k < linha_dados; k++)
			{
				if (i == agd.posicaoGD[al][k] && agd.posicaoGD[al][k] != 0)
				{
					verifica = true;
				}
			}
			//adiciona ou nao
			if (!verifica)
			{
				adjgd.push_back(i);
				break;
			}
		}
	}
	
	//começa a alocação dos GDs
	float resultado = 0.0;
	for (int i = 0; i < adjgd.size(); i++)
	{
		agd.gd_anteriores(); //salva gd anteriores

		//atualiza gds
		agd.posicaoGD[al][pgd] = adjgd[i];
		agd.quantGD[adjgd[i]] = agd.quantGD[pos_gd];
		agd.quantGD[pos_gd] = 0;

		//calcula
		resultado = fo.calculo_funcao_objetivo(false, al);
		if (resultado < incumbentv2)
		{
			return resultado;
		}
		else
		{
			agd.volta_gd_anteriores();
		}
	}
	return incumbentv2;
}

float VND::v3_VND(int ch3, float incumbentv3)
{
	//localizando chave
	int cont = 0;
	int al = 0;
	int pch = 0;
	int pos_ch = 0;

	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (ac.posicaochaves[i][j] != 0)
			{
				//incrementa contador das chaves
				cont++;
			}
			if (cont == ch3)
			{
				//encontrou a chave: pegar dados
				al = i;
				pch = j;
				pos_ch = ac.posicaochaves[i][j];
				cont++; //para nao voltar nesse loop
			}
		}
	}

	//possibilidades dos vizinhos mais proximos a candidato de alocação
	vector<int>adjch;
	bool pula = true;
	for (int i = pos_ch + 1; i < linha_dados; i++)
	{
		if (ps.noi[i] > 999) //protecao
		{
			break;
		}
		if (ps.candidato_CH[i] == 1)
		{
			if (pula)
			{
				pula = false;
				continue;
			}
			//ver se ja possui chave
			bool verifica = false;
			for (int k = 1; k < linha_dados; k++)
			{
				if (i == ac.posicaochaves[al][k] && ac.posicaochaves[al][k] != 0)
				{
					verifica = true;
				}
			}
			//adiciona ou nao
			if (!verifica)
			{
				adjch.push_back(i);
				break;
			}
		}
	}

	pula = true;
	for (int i = pos_ch - 1; i > 1; i--)
	{
		if (ps.noi[i] > 999) //protecao
		{
			break;
		}
		if (ps.candidato_CH[i] == 1)
		{
			if (pula)
			{
				pula = false;
				continue;
			}
			//ver se ja possui chave
			bool verifica = false;
			for (int k = 1; k < linha_dados; k++)
			{
				if (i == ac.posicaochaves[al][k] && ac.posicaochaves[al][k] != 0)
				{
					verifica = true;
				}
			}
			//adiciona ou nao
			if (!verifica)
			{
				adjch.push_back(i);
				break;
			}
		}
	}

	//começa a alocação
	float resultado = 0.0;
	for (int i = 0; i < adjch.size(); i++)
	{
		ac.chaves_anteriores(); //salva chaves anteriores

		//atualiza chaves
		ac.posicaochaves[al][pch] = adjch[i];
		ac.chi[al][pch] = ps.noi[adjch[i]];
		ac.chf[al][pch] = ps.nof[adjch[i]];

		//calcula
		resultado = fo.calculo_funcao_objetivo(false, al);

		if (resultado < incumbentv3)
		{
			return resultado;
		}
		else
		{
			ac.volta_chaves_anteriores();
			ac.secoes_alimentador();
		}
	}

	return incumbentv3;
}

float VND::v4_VND(int gd4, float incumbentv4)
{
	//mover GD para a posição vizinha das vizinhas

	//localiza GD
	int al = 0;
	int pgd = 0;
	int pos_gd = 0;
	int cont = 0;
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (agd.posicaoGD[i][j] != 0)
			{
				//incrementa contador de gds
				cont++;
			}
			if (cont == gd4)
			{
				//encontrou a chave: pegar dados
				al = i;
				pgd = j;
				pos_gd = agd.posicaoGD[al][pgd];
				cont++; //para nao voltar nesse loop
			}
		}
	}

	//possibilidades dos vizinhos mais proximos a candidato de alocação
	vector<int>adjgd;
	bool pula = true;
	for (int i = pos_gd + 1; i < linha_dados; i++)
	{
		if (ps.noi[i] > 999) //protecao
		{
			break;
		}
		if (ps.candidato_GD[i] == 1)
		{
			if (pula)
			{
				pula = false;
				continue;
			}
			//ver se ja possui chave
			bool verifica = false;
			for (int k = 1; k < linha_dados; k++)
			{
				if (i == agd.posicaoGD[al][k] && agd.posicaoGD[al][k] != 0)
				{
					verifica = true;
				}
			}
			//adiciona ou nao
			if (!verifica)
			{
				adjgd.push_back(i);
				break;
			}
		}
	}

	pula = true;
	for (int i = pos_gd - 1; i > 1; i--)
	{
		if (ps.noi[i] > 999) //protecao
		{
			break;
		}
		if (ps.candidato_GD[i] == 1)
		{
			if (pula)
			{
				pula = false;
				continue;
			}
			//ver se ja possui chave
			bool verifica = false;
			for (int k = 1; k < linha_dados; k++)
			{
				if (i == agd.posicaoGD[al][k] && agd.posicaoGD[al][k] != 0)
				{
					verifica = true;
				}
			}
			//adiciona ou nao
			if (!verifica)
			{
				adjgd.push_back(i);
				break;
			}
		}
	}

	//começa a alocação
	float resultado = 0.0;
	for (int i = 0; i < adjgd.size(); i++)
	{
		ac.chaves_anteriores(); //salva chaves anteriores

		//atualiza gds
		agd.posicaoGD[al][pgd] = adjgd[i];
		agd.quantGD[adjgd[i]] = agd.quantGD[pos_gd];
		agd.quantGD[pos_gd] = 0;

		//calcula
		resultado = fo.calculo_funcao_objetivo(false, al);

		if (resultado < incumbentv4)
		{
			return resultado;
		}
		else
		{
			agd.volta_gd_anteriores();
		}
	}

	return incumbentv4;
}

float VND::VND_intensificacao(int ch, int gd, float sol_incumbent)
{
	//O VND tem o objetivo de intensificar a busca do algoritmo

	float vnd_incumbent = 0.0;
	float vnd_current = 0.0;

	vnd_incumbent = sol_incumbent;
	vnd_current = sol_incumbent;

inicioVND:

	vnd_current = v1_VND(ch, vnd_incumbent);

	if (vnd_current < vnd_incumbent)
	{
		vnd_incumbent = vnd_current;
		q_vnd1++;

		//cout << "\t _ 1-VND: " << vnd_incumbent << endl;

		goto inicioVND;
	}
	
	vnd_current = v2_VND(gd, vnd_incumbent);

	if (vnd_current < vnd_incumbent)
	{
		vnd_incumbent = vnd_current;
		q_vnd2++;

		//cout << "\t _ 2-VND: " << vnd_incumbent << endl;

		goto inicioVND;
	}

	//segunda vizinhança do VND
	vnd_current = v3_VND(ch, vnd_incumbent);

	if (vnd_current < vnd_incumbent)
	{
		vnd_incumbent = vnd_current;
		q_vnd3++;

		//cout << "\t _ 3-VND: " << vnd_incumbent << endl;

		goto inicioVND;
	}
	
	vnd_current = v4_VND(gd, vnd_incumbent);

	if (vnd_current < vnd_incumbent)
	{
		vnd_incumbent = vnd_current;
		q_vnd4++;

		//cout << "\t _ 4-VND: " << vnd_incumbent << endl;

		goto inicioVND;
	}

	//fim vnd
	return vnd_incumbent;
}

float RVNS::v1_RVNS(float incumbentmain1)
{
	//DESCRICAO: selecionar 'n' chaves aleatorias do sistema e 'n' GDs e fazer o VND

	int qch = 0;
	int qgd = 0;
	int numero = 0;
	float resultadov1 = 0.0;

	//sorteios - n é limitado entre 1 e 10
	numero = rand() % 10 + 1;

	//sorteando
	for (int i = 0; i < numero; i++)
	{
		qch = rand() % ac.numch_SIS + 1;
		qgd = rand() % agd.qntGD_SIS + 1;

		resultadov1 = vnd.VND_intensificacao(qch, qgd, incumbentmain1);

		if (resultadov1 < incumbentmain1)
		{
			return resultadov1;
		}
	}
	return incumbentmain1; //nao encontra melhor solução
}

float RVNS::v2_RVNS(float incumbentmain2)
{
	//variaveis locais
	int al = 0;
	int chaves_i = 0;
	int chaves_f = 0;
	int gd = 0;
	float resultadov2 = 0.0;

	//executar vizinhança
	al = rand() % (num_AL-1) + 1;

	for (int i = 1; i < num_AL; i++)
	{
		if (i < al)
		{
			chaves_i += ac.numch_AL[i];
		}
		else if (i == al)
		{
			chaves_f = chaves_i + ac.numch_AL[i];
		}
		else
		{
			break;
		}
	}

	for (int i = chaves_i + 1; i < chaves_f + 1; i++)
	{
		gd = rand() % agd.qntGD_SIS + 1;

		resultadov2 = vnd.VND_intensificacao(i, gd, incumbentmain2);

		if (resultadov2 < incumbentmain2)
		{
			return resultadov2;
		}
	}

	return incumbentmain2;
}

float RVNS::v2_RVNS_AUX(float incumbentmain2, int al)
{
	//variaveis locais
	int chaves_i = 0;
	int chaves_f = 0;
	int gd = 0;
	float resultadov2 = 0.0;

	//executar vizinhança
	for (int i = 1; i < num_AL; i++)
	{
		if (i < al)
		{
			chaves_i += ac.numch_AL[i];
		}
		else if (i == al)
		{
			chaves_f = chaves_i + ac.numch_AL[i];
		}
		else
		{
			break;
		}
	}

	for (int i = chaves_i + 1; i < chaves_f + 1; i++)
	{
		gd = rand() % agd.qntGD_SIS + 1;

		resultadov2 = vnd.VND_intensificacao(i, gd, incumbentmain2);

		if (resultadov2 < incumbentmain2)
		{
			return resultadov2;
		}
	}

	return incumbentmain2;
}

float RVNS::v3_RVNS(float incumbentmain3)
{
	//variaveis locais
	int al = 0;
	int gd_i = 0;
	int gd_f = 0;
	int ch = 0;
	float resultadov3 = 0.0;

	//executar vizinhança
	al = rand() % (num_AL-1) + 1;

	for (int i = 1; i < num_AL; i++)
	{
		if (i < al)
		{
			gd_i += agd.numgd_AL[i];
		}
		else if (i == al)
		{
			gd_f = gd_i + agd.numgd_AL[i];
		}
		else
		{
			break;
		}
	}

	for (int i = gd_i + 1; i < gd_f + 1; i++)
	{
		ch = rand() % ac.numch_SIS + 1;

		resultadov3 = vnd.VND_intensificacao(ch, i, incumbentmain3);

		if (resultadov3 < incumbentmain3)
		{
			return resultadov3;
		}
	}

	return incumbentmain3;
}

float RVNS::v3_RVNS_AUX(float incumbentmain3, int al)
{
	//variaveis locais
	int gd_i = 0;
	int gd_f = 0;
	int ch = 0;
	float resultadov3 = 0.0;

	//executar vizinhança
	for (int i = 1; i < num_AL; i++)
	{
		if (i < al)
		{
			gd_i += agd.numgd_AL[i];
		}
		else if (i == al)
		{
			gd_f = gd_i + agd.numgd_AL[i];
		}
		else
		{
			break;
		}
	}

	for (int i = gd_i + 1; i < gd_f + 1; i++)
	{
		ch = rand() % ac.numch_SIS + 1;

		resultadov3 = vnd.VND_intensificacao(ch, i, incumbentmain3);

		if (resultadov3 < incumbentmain3)
		{
			return resultadov3;
		}
	}

	return incumbentmain3;
}

float RVNS::v4_RVNS(float incumbentmain4)
{
	//quarta vizinhança - adiciona um GD

	//variaveis locais
	int opcao = 0;
	int eqpto = 0;
	int al = 0;

	//executa vizinhança
	//adiciona GD
inicioopGD:
	eqpto = (rand() % linha_dados - 1) + 1;

	if (ps.candidato_GD[eqpto] == 0 || agd.quantGD[eqpto] >= maxGD)
	{
		goto inicioopGD;
	}

	//identifica o alimentador
	al = gvns.localizaAL(eqpto);

	//adiciona GD
	bool nova_posicao = false;

	for (int i = 1; i < linha_dados; i++)
	{
		if (agd.posicaoGD[al][i] == eqpto)
		{
			//ja tem a posicao, somente adiciona o GD nela
			nova_posicao = false;
			agd.quantGD[eqpto]++;
			break;
		}
		else if (agd.posicaoGD[al][i] == 0)
		{
			nova_posicao = true;
			agd.posicaoGD[al][i] = eqpto; //adiciona gd na posicao
			agd.quantGD[eqpto]++; //incrementa quantidade de GD na barra
			break;
		}
	}
	agd.numgd_AL[al]++; //adiciona o gd nos contadores do alimentador
	agd.qntGD_SIS++; //adiciona o gd nos contadores do sistema

	//executar vizinhança 3 no alimentador que o gd foi selecionada
	float resultado = 0.0;
	resultado = rvns.v3_RVNS_AUX(incumbentmain4, al);

	if (resultado < incumbentmain4)
	{
		//retorna valor
		return resultado;
	}
	else
	{
		//exclui posicao do GD
		if (nova_posicao)
		{
			for (int i = 1; i < linha_dados; i++)
			{
				if (agd.posicaoGD[al][i + 1] == 0)
				{
					agd.posicaoGD[al][i] = 0; //exclui gd na posicao
					break;
				}
			}
		}
		agd.numgd_AL[al]--; //retira o gd nos contadores
		agd.qntGD_SIS--;
		agd.quantGD[eqpto]--; //retira gd inserido

		//retorna valor anterior
		return incumbentmain4;
	}
}

float RVNS::v5_RVNS(float incumbentmain5)
{
	int eqpto = 0;

	//adiciona Chave de manobra
inicioopCH:
	eqpto = (rand() % linha_dados - 1) + 1;

	if (ps.candidato_CH[eqpto] == 0)
	{
		goto inicioopCH;
	}
	else
	{
		for (int i = 1; i < num_AL; i++)
		{
			for (int j = 1; j < linha_dados; j++)
			{
				if (ac.posicaochaves[i][j] == eqpto)
				{
					goto inicioopCH;
				}
			}
		}
	}

	//identifica o alimentador
	int al = 0;
	al = gvns.localizaAL(eqpto);

	//adciona a chave
	for (int i = 1; i < linha_dados; i++)
	{
		if (ac.posicaochaves[al][i] == 0)
		{
			ac.posicaochaves[al][i] = eqpto;
			ac.chi[al][i] = ps.noi[ac.posicaochaves[al][i]];
			ac.chf[al][i] = ps.nof[ac.posicaochaves[al][i]];
			ac.numch_AL[al]++;
			ac.numch_SIS++;
			break;
		}
	}

	//executa a vizinhança 2 do RVNS no alimentador que a chave foi adicionada
	float resultado = 0.0;
	resultado = rvns.v2_RVNS_AUX(incumbentmain5, al);
	
	if (resultado < incumbentmain5)
	{
		//retorna valor
		return resultado;
	}
	else
	{
		//exclui chave
		for (int i = 1; i < linha_dados; i++)
		{
			if (ac.posicaochaves[al][i + 1] == 0)
			{
				ac.posicaochaves[al][i] = 0;
				ac.chi[al][i] = 0;
				ac.chf[al][i] = 0;
				ac.numch_AL[al]--;
				ac.numch_SIS--;
				break;
			}
		}

		return incumbentmain5; //nao obteve melhor resultado
	}
}

//############################################################################################

void DELETA()
{
	//deleta todas as variaveis da solução para começar uma nova simulação
	for (int i = 0; i < num_AL; i++)
	{
		fo.fo_al[i] = 0;
		fo.fo_al_save[i] = 0;
		ac.numch_AL[i] = 0;
		agd.numgd_AL[i] = 0;

		for (int j = 0; j < linha_dados; j++)
		{
			
			ac.posicaochaves[i][j] = 0;
			ac.antchf[i][j] = 0;
			ac.antchi[i][j] = 0;
			ac.antpos[i][j] = 0;
			ac.chf[i][j] = 0;
			ac.chi[i][j] = 0;
			

			
			agd.antGD[i][j] = 0;
			agd.posicaoGD[i][j] = 0;
			agd.quantGD[j] = 0;
			agd.ant_quantGD[j] = 0;
		}
	}

	agd.qntGD_SIS = 0;
	ac.numch_SIS = 0;
}

//############################################################################################

int main()
{

	srand(static_cast <unsigned int> (time(NULL)));	//faz a aleatoriedade com base no relogio

	int itGVNS = 0;
	int simulacao = 0;

inicio_alg:

	simulacao++;

	cout << "Simulacao: " << simulacao << endl;
	cout << "\n";

	//variaveis a serem analisadas
	float current_solution = 0.0;
	float incumbent_solution = 0.0;

	rvns.q_rvns1 = 0;
	rvns.q_rvns2 = 0;
	rvns.q_rvns3 = 0;
	rvns.q_rvns4 = 0;
	vnd.q_vnd1 = 0;
	vnd.q_vnd2 = 0;

	//faz a leitura dos parametros do circuito e dos cenarios
	ps.leitura_parametros();
	ps.leituraCenarios();

	//somatorio da potencia total do sistema
	ps.somatorio_potencia();

	//resolve o fluxo de potencia
	fxp.fluxo_potencia(false);

	//tirando de pu, para conferir - imprime o fluxo de potencia
	//fxp.valores_nominais_tensao();

	for (int i = 1; i < num_AL; i++)
	{
		ac.numch_AL[i] = SWinicial;
		agd.numgd_AL[i] = GDinicial;
	}

	//primeira alocacao: esta eh feita de forma aleatoria
	gvns.primeiraaloc();

	//iniciando do GVNS

	//suposicao para o alimentador 1: TESTE
	/*
	ac.posicaochaves[1][1] = 7;
	ac.posicaochaves[1][2] = 9;
	ac.posicaochaves[1][3] = 14;
	*/


	//primeiras chaves
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			ac.chi[i][j] = ps.noi[ac.posicaochaves[i][j]];
			ac.chf[i][j] = ps.nof[ac.posicaochaves[i][j]];
		}
	}

	//contado total de chaves
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (ac.chi[i][j] != 0 && ac.chf[i][j] != 0)
			{
				ac.numch_SIS++;
			}
		}
	}

	//contando total de GDs no sistema
	agd.qntGD_SIS = 0;
	for (int i = 1; i < linha_dados; i++)
	{
		if (agd.quantGD[i] != 0)
		{
			agd.qntGD_SIS = agd.qntGD_SIS + agd.quantGD[i];
		}
	}

	//imprimindo chaves iniciais:
	cout << "Chaves Iniciais:" << endl;

	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (ac.chi[i][j] != 0 && ac.chf[i][j] != 0)
			{
				cout << ac.chi[i][j];
				cout << "|--|";
				cout << ac.chf[i][j] << endl;
			}
		}
	}
	cout << "\n";

	//imprimindo GDs iniciais
	cout << "GDs Iniciais:" << endl;
	cout << "\n";
	cout << "Barras: ";
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (agd.posicaoGD[i][j] != 0)
			{
				cout << ps.nof[agd.posicaoGD[i][j]]<<", ";
			}
		}
	}
	cout << "\n\n";

	//calcular valor da FO total

	incumbent_solution = fo.calculo_funcao_objetivo(true, 0);
	cout << "---------------------------------" << endl;

	ac.chaves_anteriores();
	agd.gd_anteriores();

	current_solution = incumbent_solution;

	cout << "Solucao Inicial: " << incumbent_solution << endl;
	cout << "\n";

	itGVNS = 0;

nmgvns:

	itGVNS++;
	cout << "Processo de parada - " << itGVNS << "/" << criterio_parada << endl;
	//cout << "\n";

metaheuristicGVNS:

	//cout << "1. \n";
	current_solution = rvns.v1_RVNS(incumbent_solution);

	if (current_solution < incumbent_solution)
	{
		incumbent_solution = current_solution;
		rvns.q_rvns1++;

		cout << "1-RVNS: " << incumbent_solution << endl;

		goto metaheuristicGVNS;
	}

	//cout << "2. \n";
	current_solution = rvns.v2_RVNS(incumbent_solution);

	if (current_solution < incumbent_solution)
	{
		incumbent_solution = current_solution;
		rvns.q_rvns2++;

		cout << "2-RVNS: " << incumbent_solution << endl;

		goto metaheuristicGVNS;
	}

	//cout << "3. \n";
	current_solution = rvns.v3_RVNS(incumbent_solution);

	if (current_solution < incumbent_solution)
	{
		incumbent_solution = current_solution;
		rvns.q_rvns3++;

		cout << "3-RVNS: " << incumbent_solution << endl;

		goto metaheuristicGVNS;
	}

	//cout << "4. \n";
	current_solution = rvns.v4_RVNS(incumbent_solution);

	if (current_solution < incumbent_solution)
	{
		incumbent_solution = current_solution;
		rvns.q_rvns4++;

		cout << "4-RVNS: " << incumbent_solution << endl;

		goto metaheuristicGVNS;
	}

	//cout << "5. \n";
	current_solution = rvns.v5_RVNS(incumbent_solution);

	if (current_solution < incumbent_solution)
	{
		incumbent_solution = current_solution;
		rvns.q_rvns5++;

		cout << "5-RVNS: " << incumbent_solution << endl;

		goto metaheuristicGVNS;
	}

	if (itGVNS < criterio_parada)
	{
		cout << "\n";
		goto nmgvns;
	}

	////////////////////////////////////////////////////////////
	//Fim GVNS

	cout << "\n";

	//chaves finais
	cout << "Chaves Finais:" << endl;

	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (ac.chi[i][j] != 0 && ac.chf[i][j] != 0)
			{
				cout << ac.chi[i][j];
				cout << " |--| ";
				cout << ac.chf[i][j] << endl;
			}
		}
	}
	cout << "\n";

	//GDS Finais
	cout << "GDs Finais" << endl;
	cout << "\n";
	cout << "Barras: ";
	for (int i = 1; i < num_AL; i++)
	{
		for (int j = 1; j < linha_dados; j++)
		{
			if (agd.posicaoGD[i][j] != 0)
			{
				cout << ps.nof[agd.posicaoGD[i][j]] << ", ";
			}
		}
	}
	cout << "\n\n";

	//total iteracoes
	cout << "Numero de iteracoes: " << itGVNS << endl;
	cout << "\n";

	cout << "Operacoes nas vizinhacas:" << endl;
	cout << "rvns 1: " << rvns.q_rvns1 << endl;
	cout << "rvns 2: " << rvns.q_rvns2 << endl;
	cout << "rvns 3: " << rvns.q_rvns3 << endl;
	cout << "rvns 4: " << rvns.q_rvns4 << endl;
	cout << "rvns 5:" << rvns.q_rvns5 << endl;
	cout << "vnd 1: " << vnd.q_vnd1 << endl;
	cout << "vnd 2: " << vnd.q_vnd2 << endl;
	cout << "vnd 3: " << vnd.q_vnd3 << endl;
	cout << "vnd 4: " << vnd.q_vnd4 << endl;
	cout << "\n";

	//fluxo de potencia:
	//cout << "Numero de fluxo de potencia da simulacao: " << fxp.contadorFXP << endl;
	//cout << "\n";

	//Solucao final
	cout << "Solucao otima: " << incumbent_solution << endl;
	cout << "\n\n";

	cout << "#######################################################################################################" << endl;
	cout << "\n\n";

	if (simulacao < numero_simulacoes) 
	{
		//reseta solução e volta para o começo do algoritmo
		DELETA();
		goto inicio_alg; 
	}

	cout << "\n";
	cout << "________________ FIM." << endl;

	return 0;
}