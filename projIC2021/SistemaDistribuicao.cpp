#include "SistemaDistribuicao.h"

bool SistemaDistribuicao::elementofim(string ler)
{
    if (ler == "BARRA" || ler == "RAMO" || ler == "FIM")
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

ELEMENTO SistemaDistribuicao::VerifSUB(string sub)
{
    int num = 0;
    num = stoi(sub);

    if (num > 1000)
    {
        return SUB;
    }
    else
    {
        return nSUB;
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

            do
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

                getline(inputarq, linha, '\n');
                getline(inputarq, linha, ',');

            } while (!elementofim(linha)); break;
            //fim leitura das barras

        case RAMO:

            getline(inputarq, linha, '\n');

            getline(inputarq, linha, ',');

            do 
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

                getline(inputarq, linha, '\n');
                getline(inputarq, linha, ',');

            } while (!elementofim(linha)); break;
            //fim leitura dos ramos

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

void SistemaDistribuicao::Topologia(Circuito* pc)
{
    vector<Ramo*>auxiliar;

    //divisao dos subsistemas formado por cada alimentador
    for (int k = 0; k < pc->ramolista.size(); k++)
    {
        if (pc->ramolista[k]->estadoRamo == ON)
        {
            if (k == 0)
            {
                auxiliar.push_back(pc->ramolista[k]);
            }
            else
            {
                if (pc->ramolista[k]->pb1->tipoBarra == SE)
                {
                    ss.push_back(auxiliar);
                    auxiliar.clear();
                }
                else
                {
                    if (pc->ramolista[k]->tipoRamo != CS)
                    {
                        auxiliar.push_back(pc->ramolista[k]);
                    }
                }
            }
        }
    }

    ss.push_back(auxiliar);
    auxiliar.clear();

    //ramos de interconecao entre alimentadores
    for (int k = 0; k < pc->ramolista.size(); k++)
    {
        if (pc->ramolista[k]->tipoRamo == CS)
        {
            interlig.push_back(pc->ramolista[k]);
        }
    }


}
