#include "SistemaDistribuicao.h"

bool SistemaDistribuicao::elementofim(string ler)
{
    if (ler == "BARRA" || ler == "RAMO")
    {
        return true;
    }
    else
    {
        return false;
    }
}

ELEMENTO SistemaDistribuicao::NomeElemento(string nome)
{
    if (nome == "BARRA")
    {
        return BARRA;
    }
    else if (nome == "RAMO")
    {
        return RAMO;
    }
}

Circuito* SistemaDistribuicao::DadosSistema(string arq)
{
    ifstream inputarq;
    string linha;
    Barra* pBarra;
    Ramo* pRamo;

    Circuito* pCircuito = new Circuito();

    inputarq.open(arq);

    if (!inputarq.is_open())
    {
        return nullptr;

    }

    getline(inputarq, linha, ',');

    while (linha != "FIM")
    {
        switch (this->NomeElemento(linha))
        {
        case BARRA:

            getline(inputarq, linha, '\n');

            getline(inputarq, linha, ',');

            while (!elementofim(linha))
            {
                pBarra = new Barra(); //cria um objeto barra

                pBarra->id = stoi(linha); //numero da barra

                getline(inputarq, linha, ',');
                pBarra->Sbus.real(stof(linha)); //potencia ativa

                getline(inputarq, linha, ',');
                pBarra->Sbus.imag(stof(linha)); //potencia reativa

                getline(inputarq, linha, ',');
                pBarra->TipoBarra(linha); //tipo da barra

                pCircuito->barralista.push_back(pBarra); //armazena a barra no vetor de barras

                getline(inputarq, linha, ',');
            }
            //fim da adicao de barras

        case RAMO:

            getline(inputarq, linha, '\n');

            getline(inputarq, linha, ',');

            while (!elementofim(linha))
            {
                pRamo = new Ramo();

                pRamo->setBarra1(pCircuito->getBarra(stoi(linha))); //no inicial

                getline(inputarq, linha, ',');
                pRamo->setBarra2(pCircuito->getBarra(stoi(linha))); //no final

                getline(inputarq, linha, ',');
                pRamo->Zl.real(stof(linha)); //resistencia

                getline(inputarq, linha, ',');
                pRamo->Zl.imag(stof(linha)); //reatancia

                getline(inputarq, linha, ',');
                pRamo->km = stof(linha); //distancia

                getline(inputarq, linha, ',');
                pRamo->TipoRamo(linha); //tipo do ramo

                getline(inputarq, linha, ',');
                pRamo->EstadoRamo(linha); //se esta ligado ou desligado

                pCircuito->ramolista.push_back(pRamo); //armazena o ramo no vetor de ramos

                getline(inputarq, linha, ',');
            }

        default:
            break;
        }
    }

    if (pCircuito)
    {
        return pCircuito;
    }
    else
    {
        return nullptr;
    }
}
