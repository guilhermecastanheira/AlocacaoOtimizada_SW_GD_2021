//Programa principal


#include <iostream>
#include "SistemaDistribuicao.h"
#include "FluxoPotencia.h"

//PARAMETROS

complex<float> Vi = 13800.00; //tensao inicial nas barras
const float Vbase = 13800.00;
const float Sbase = 100*pow(10,3);
const float tolerancia_pflow = 0.00001; //tolerancia do fluxo de potencia


int main()
{
    SistemaDistribuicao* pSD = new SistemaDistribuicao();
    Circuito* pC;
    pC = pSD->DadosSistema("sistematesteEX.txt");

    pSD->Topologia(pC); //monta os subsistemas
    


    FluxoPotencia* pFluxo = new FluxoPotencia(pSD, tolerancia_pflow, Vi, Vbase, Sbase);
    pFluxo->resultado(pSD);
    
    //fazer o fluxo de cargas

    //fazer a metaheuristica

    //definir multiobjetivos no sistema

    delete pSD;
    delete pC;
    return 0;
}