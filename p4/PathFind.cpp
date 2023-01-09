// ###### Config options ################

#define PRINT_PATHS 1

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1
               
#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

#ifdef PRINT_PATHS
#include "ppm.h"
#endif

using namespace Asedio;

Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight){ 
    return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0); 
}

//Funcion de comparacion necesaria para el funcionamiento de la ordenacion del monticulo (lo ordenamos respecto al valor de F)
bool comp(const AStarNode* A,const AStarNode* B) 
{
	return A->F < B->F;
}

//Funcion usada para ver si un nodo particular se encuentra en el vector 
bool estaEnVect(AStarNode* A,std::vector<AStarNode*> V)
{
	bool esta = false; //suponemos que no esta en el vector
	for(std::vector<AStarNode*>::iterator it = V.begin() ; it != V.end() ; it++) //for para recorrerlo
	{	
		if((*it)->position.x == A->position.x && (*it)->position.y == A->position.y) //si lo encontramos
			esta = true; //ponemos esta a true
	}
	return esta;
}

//añadiremos una coste adicional a aquellas posiciones que esten en el rango de una de las defensas
void DEF_LIB_EXPORTED calculateAdditionalCost(float** additionalCost
                   , int cellsWidth, int cellsHeight, float mapWidth, float mapHeight
                   , List<Object*> obstacles, List<Defense*> defenses) 
{
	float cost = 0;
	
    float cellWidth = mapWidth / cellsWidth;
    float cellHeight = mapHeight / cellsHeight;

    for(int i = 0 ; i < cellsHeight ; ++i) 
    {
        for(int j = 0 ; j < cellsWidth ; ++j) //recorrer casillas
        {
            Vector3 cellPosition = cellCenterToPosition(i, j, cellWidth, cellHeight); //obtener la posicion de la casilla
			for(List<Defense*>::iterator it = defenses.begin() ; it != defenses.end() ; it++) //recorrer defensas
			{
				//si la distancia entre la casilla y al defensa es menor que su rango, entonces estamos dentro de su rango
				if(_sdistance(cellPosition,(*it)->position) <  (*it)->range)//le añadiremos un coste relacionado con las propiedades de la defensa
					cost += (0.5*(*it)->damage + 0.3*(*it)->health + 0.2*(*it)->attacksPerSecond); //valoraremos mas el daño de la defensa, luego su salud y por ultimo su ataque por segundo
			}
			additionalCost[i][j] = cost; //asignamos coste a dicha casilla
			cost = 0;	//reiniciamos el valor de coste para la siguiente
        }
    }
}

void DEF_LIB_EXPORTED calculatePath(AStarNode* originNode, AStarNode* targetNode
                   , int cellsWidth, int cellsHeight, float mapWidth, float mapHeight
                   , float** additionalCost, std::list<Vector3> &path) 
{
    float totalCost = 0; //coste del camino dsde un punto a otro
    AStarNode* current = originNode; //nodo que usamos para movernos entre padres e hijos
    
	std::vector<AStarNode*>  Nopen,Nclosed;	//vectores para almacenar los nodos abiertos y cerrados
	
	current->parent = NULL; //ponemos la referencia al padre del nodo origen a NULL ya que partimos de el
	
	Nopen.push_back(current); //añadimos el nodo origen al vector de abiertos
	
	std::make_heap(Nopen.begin(),Nopen.end()); //creamos un monticulo sobre este vector para obtener una ordenacion rapida
    
    while(!Nopen.empty()) //mientras que el vector de abiertos no este vacio:
    {
    
    	while(current != targetNode && current != NULL && !Nopen.empty()) //mientras que el vector de abiertos no este vacio ni current sea (el nodo obj o null):
    	{
	       	current = Nopen.front(); //Obtenemos el mejor nodo de la lista de abiertos
	       	std::pop_heap(Nopen.begin(),Nopen.end()); //traemos al final del vector el elemento a eliminar
	       	Nopen.pop_back(); //lo eliminamos

			for (List<AStarNode*>::iterator it=current->adjacents.begin(); it != current->adjacents.end(); ++it) //for para recorrer los adyacentes del nodo actual
			{
				if(!estaEnVect((*it),Nopen) && !estaEnVect((*it),Nclosed)) //si estos no estan en los vectores de nodos abiertos o cerrados
				{
					(*it)->H = _sdistance((*it)->position,targetNode->position); //evaluamos la funcion heuristica (coste en llegar a la solucion)
					(*it)->G = _sdistance((*it)->position,originNode->position); //evaluamos la funcion g (coste en llegar a ese nodo especifico)
					(*it)->F = (*it)->H + (*it)->G + additionalCost[(int)((*it)->position.x / cellsWidth)][(int)((*it)->position.y / cellsHeight)]; //f valdra g + h y le añadimos un coste adicional determinado por la funcion calculateAdditionalCost
					(*it)->parent = current; //asignamos su respectivo padre
					Nopen.push_back((*it)); //lo metemos en la lista de abiertos
				}
			}
			
			Nclosed.push_back(current); //una vez abierto todos sus posibles adyacentes, lo movemos a la lista de cerrados
			std::sort_heap(Nopen.begin(),Nopen.end()); //ordenamos el monticulo
    	}
    
    	if(current == targetNode) //si hemos llegado al nodo objetivo:
     	{
     		while(current != originNode)//recorremos el camino a la inversa para ver el coste de dicha solucion (mientras que no lleguemos al nodo origen)
     		{
     			totalCost += _sdistance(current->position,current->parent->position); //añadimos el coste de llegar hacia el
     			path.push_front(current->position); //añadimos la posicion de dicho nodo al camino
     			current = current->parent; //cambiamos el nodo actual por por su padre
			}
    	}	
    }
}
