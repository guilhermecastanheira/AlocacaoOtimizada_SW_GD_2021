#pragma once

#include <iostream>
#include <tuple>

#define LIGADO 1
#define DESLIGADO 0

#define sistema 135 //quantas barras tem no sistema

using namespace std;

tuple<int, int> localizaSecao(int barra[1000], int alimentador, int linhas)
{
    int contador, menor, maior;

    contador = 0;

    for (int i = 1; i < linhas; i++)
    {
        contador++;

        if (barra[i] == alimentador)
        {
            menor = contador;
        }

        if (barra[i] > alimentador || i > sistema)
        {
            maior = contador - 1;
            break;
        }

    }

    return make_tuple(menor, maior);
}

int posicaoBarra(int barra, int barras[1000], int linhas)
{
    int posicao = 0;

    for (int i = 1; i < linhas; i++)
    {
        if (barra == barras[i])
        {
            posicao = i;
        }
    }

    return posicao;
}

void voltaEstadoVanila(int pos[1000], int pos_original[1000], int linhas)
{
    for (int i = 1; i < linhas; i++)
    {
        pos[i] = pos_original[i];
    }
}

int comparacaoCAMADASECAO(int camada[157][157], vector<int>secao, int linhas)
{
    int contador = 0;

    for (int i = 0; i < secao.size(); i++)
    {
        for (int j = 1; j < linhas; j++)
        {
            for (int k = 1; k < linhas; k++)
            {
                if (secao[i] == camada[j][k])
                {
                    contador++;
                }
            }
        }
    }

    return contador;
}

void posREMANEJAMENTO(vector<int>* pos, int barra, int noinicial[1000], int nofinal[1000], int linhas)
{
    //procura a chave de manobra
    for (int i = sistema + 1; i < linhas; i++)
    {
        if (barra == noinicial[i] || barra == nofinal[i])
        {
            pos->push_back(i);
            break;
        }
    }
}

int locSE(int inicial, int final, int noinicial[1000], int nofinal[1000], vector<int>sec)
{
    int SE = 0;
    int barra_outra_SE = 0;

    //qual no esta na secao analisada?
    for (int i = 1; i < sec.size(); i++)
    {
        if (inicial == sec[i])
        {
            barra_outra_SE = final;
            break;
        }
        else if (final == sec[i])
        {
            barra_outra_SE = inicial;
            break;
        }
    }

    for (int i = 1; i < sistema; i++)
    {
        if (noinicial[i] >= 1000)
        {
            SE = noinicial[i];
        }

        if (nofinal[i] == barra_outra_SE)
        {
            break;
        }
    }

    return SE;
}
