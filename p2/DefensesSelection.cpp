// ###### Config options ################


// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1
               
#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

using namespace Asedio;

//estructura para almacenar los candidatos
struct candidato
{
    Defense* def;	//puntero a defensa
    float valor;	//valor que posee dicha defensa
    candidato(Defense* d,float v): def(d),valor(v){} //constructor
};

//operador para llevar a cabo la ordenacion de candidatos respecto al coste.
bool operator<(candidato c1,candidato c2)
{
    return (c1.def->cost < c2.def->cost);
}

//cabecera de la funcion que da valor a las defensas
float DefenseValue(std::list<Defense*>::iterator currentDef);


void DEF_LIB_EXPORTED selectDefenses(std::list<Defense*> defenses, unsigned int ases, std::list<int> &selectedIDs
            , float mapWidth, float mapHeight, std::list<Object*> obstacles)             
{
    //definicion de variables necesarias
    //creamos vector de candidatos
    std::vector<candidato> Candidatos; 
    //iterador para recorrer las defensas
    std::list<Defense*>::iterator defit = defenses.begin(); 
    
    //No contamos el centro de extraccion ya 
    //que este es necesario para el funcionamiento
    //metemos la primera defensa(centro de extraccion)
    selectedIDs.push_back((*defit)->id);
    //pasamos a la siguiente defensa
    defit++;


    //bucle para inicializar la lista de candidatos
    while(defit != defenses.end()) 
    {   
     	//Cada candidato tendra un puntero a la defensa y valor que tiene la misma defensa
        Candidatos.push_back(candidato((*defit),DefenseValue(defit)));
        defit++;//pasamos a la siguiente defensa
    }
    //ordenamos el vector (de menor a mayor coste)
    std::sort(Candidatos.begin(),Candidatos.end());
    

    //utilizaremos una matriz de flotantes para poder representar la tabla de subproblemas resuletos
    //ademas, esta debera ser de dimension 
    //numdecandidatos(defensas) * presupuesto + 1 (con el objetivo de poder llegar al valor de ases
    float evaluacionTotal[Candidatos.size()][ases+1]; //definicion de tabla de subproblemas

	//Bucle para recorrer la tabla de subproblemas
    for(int i = 0 ; i < Candidatos.size() ; i++)
    {
        for(int j = 0 ; j < ases+1 ; j++)
        {
        	//si estamos en la 1ª fila y j (presupuesto actual) es mayor o igual a 
        	//lo que me cuesta la defensa, entonces la meto en la "mochila"
            if(i == 0 && j >= Candidatos[i].def->cost)	
                evaluacionTotal[i][j] = Candidatos[i].valor;
                
            //si estamos en la 1ª fila y j (presupuesto actual) es menor a
            //lo que me cuesta la defensa, entonces no meto nada en la mochila
            if(i == 0 && j < Candidatos[i].def->cost)
                evaluacionTotal[i][j] = 0;
                
            //si estamos en una fila superior a la 1ª
            if(i > 0)
            {
            	//si j (presupuesto actual) es menor a el coste de la defensa actual
            	//entonces no puedo meter la defensa actual (me quedo igual que en la fila anterior)
                if(j < Candidatos[i].def->cost)
                    evaluacionTotal[i][j] = evaluacionTotal[i-1][j];
                    
                //si j (presupuesto actual) es mayor o igual a el coste de la defensa actual
                //entonces nos quedamos con el maximo de lo que tenia antes(fila anterior) y 
                //lo que tendria su meto en la mochila si meto la defensa actual en la mochila
                else
                    evaluacionTotal[i][j] = std::max(evaluacionTotal[i-1][j],
                    evaluacionTotal[i-1][j-Candidatos[i].def->cost] + Candidatos[i].valor);
            }
        }
    }
    //el maximo valor disponible para el conjunto de defensas estaria en la ultima posicion
    //tanto de fila y columna de la tabla de subproblemas resueltos 
    

	//variables para recorrer hacia atras los resultados de la mochila
    int j = ases, i = Candidatos.size() - 1;
    //lista de defensas seleccionadas (defensas que esten en la mochila)
    std::list<Defense*> DefensasSeleccionadas;
    //mientras j (el presupuesto) sea mayor que 0 y la fila sea mayor igual a 1
    while(j > 0 && i >= 1)
    {
    	//si la posicion actual es distinta a la superior (se ha metido la defensa)
        if(evaluacionTotal[i][j] != evaluacionTotal[i-1][j]) 
        {
        	//guardamos la defensa
            DefensasSeleccionadas.push_back(Candidatos[i].def);
            //al presupuesto se le quita lo que nos costo la defensa
            j = j - Candidatos[i].def->cost; 
        }   
        i--; //pasamos a la fila superior
    }

	//Ahora insertaremos las IDs de las defensas seleccionadas
    std::list<Defense*>::iterator dit = DefensasSeleccionadas.begin();
    while(dit != DefensasSeleccionadas.end())
    {
        selectedIDs.push_back((*dit)->id);
        dit++;
    }
}

/*
Para realizar la funcion que dara un valor a cada defensa, deberemos tener en cuenta los atributos de esta, intentando sacar
los maximos de estos en relación a su coste.
Los atributos que dan valor a la defensa son: los ataques por segundo (attacksPerSecond), el daño (damage), la dispersion (dispersion), la vida (health), el radio (radio)
El atributo que resta valor a la defensa es su coste (cost)
Entonces haremos un promedio devolviendo la suma de los atributos que dan valor a la defensa entre su coste
*/

float DefenseValue(std::list<Defense*>::iterator currentDef)
{
	return ((*currentDef)->health + (*currentDef)->attacksPerSecond + (*currentDef)->damage + (*currentDef)->dispersion)/(*currentDef)->cost;
}
