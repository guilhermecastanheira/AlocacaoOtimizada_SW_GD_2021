//Programa principal

#include <iostream>
#include "SistemaDistribuicao.h"


int main()
{
    SistemaDistribuicao* pSD = new SistemaDistribuicao();
    Circuito* pC;
    pC = pSD->DadosSistema("sistematesteEX.txt");

    //fazer o fluxo de cargas

    //fazer a metaheuristica

    //definir multiobjetivos no sistema

    delete pSD;
    delete pC;
    return 0;
}