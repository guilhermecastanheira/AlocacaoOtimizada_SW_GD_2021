#include <iostream>
#include "SistemaDistribuicao.h"


int main()
{
    SistemaDistribuicao* pSD = new SistemaDistribuicao();
    Circuito* pC;
    pC = pSD->DadosSistema("sistematesteEX.txt");

    delete pSD;
    delete pC;
    return 0;
}