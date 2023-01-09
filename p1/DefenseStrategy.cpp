// ###### Config options ################

//#define PRINT_DEFENSE_STRATEGY 1    // generate map images

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

#ifdef PRINT_DEFENSE_STRATEGY
#include "ppm.h"
#endif


#ifdef CUSTOM_RAND_GENERATOR
RAND_TYPE SimpleRandomGenerator::a;
#endif

using namespace Asedio;

//Estructura de Celda, formada por una fila, columna y un valor (cuanto de centrada es la celda)
struct Celda
{
	int row,col;
	float valor;
	Celda(int r, int c, int v): row(r), col(c), valor(v) {}
};

//operador de menor para hacer poder ordenar las celdas en la lista en referencia a su valor
bool operator<(const Celda& c1,const Celda& c2)
{
	return c1.valor < c2.valor;
} 

//Funciones proporcionadas:
//Pasar de celda a posicion y de posicion a celda
Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight){ return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0); }
void positionToCell(const Vector3 pos, int &i_out, int &j_out, float cellWidth, float cellHeight){ i_out = (int)(pos.y * 1.0f/cellHeight); j_out = (int)(pos.x * 1.0f/cellWidth); }

//Funcion de factibilidad
bool EsFactible(int row, int col,List<Defense*>::iterator currentdefense, int nCellsWidth, int nCellsHeight 
	, float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses, int numDefensasColocadas);

//Funcion de cellValue
float cellValueExtrationCenter(int row, int col, bool** freeCells,float cellWidth , float cellHeight,int nCellsWidth, int nCellsHeight)
//Para poder evaluar cada celda en el caso de querer construir el centro de Extraccion y las defensas debemos intentar
//crearlo lo mas cercano al centro con el objetivo de que la posicion disponible mas centrica sea el centro de 
//extraccion para poder construir el numero maximo de defensas alrededor suya
{
	if(freeCells[row][col])// Comprobamos que la celda que hemos seleccionado esta libre
	{
		//Debemos tener en cuenta que las casillas mas "centricas" sera las que mas se acerquen a la posicion formada por la celda situada en
		//nCellsWidth/2(la mitad del ancho) en el caso de la row y a nCellsHeight/2(la mitad de la altutra) en el caso de la col.
		//Para poder medir la distancias entre la celda que se nos pasa y la celda central conseguiremos sus posisciones reales
		//gracias a la funcion proporcionada 'cellCenterToPosition' y gracias a la funcion '_distance' podemos obtener la distancia entre ambos.
		//Por ultimo, para poder tener una ponderacion devolveremos el siguiente dato: '1000/_distance(Centro,celdaActual)' debido a que
		//cuanto mas cercana sea la celda proporcionada, menor sera la distancia entre ellas, por consecuente, mayor sera su puntuacion.
		Vector3 Centro = cellCenterToPosition(nCellsWidth/2,nCellsHeight/2,cellWidth,cellHeight);
		Vector3 celdaActual = cellCenterToPosition(row,col,cellWidth,cellHeight);
		return (float) 1000/_distance(Centro,celdaActual);
	}
	return -1; //No esta libre
}


void cellValueDefenses(std::list<Celda>& Celdas,List<Defense*>::iterator CentroDeExtraccion,// Para poder evaluar las celdas en el caso de querer construir las defensas, haremos algo similar que con el centro de extraccion
	float cellWidth, float cellHeight)//solo que ahora colocaremos las defensas lo mas cercano al centro de extraccion. Para ello modificaremos el valor de las celdas candidatas con el nuevo valor
{													
	List<Celda>::iterator cit = Celdas.begin(); //definimos iterador
	while(cit != Celdas.end())//mientras haya celdas
	{
			Vector3 celdaActual = cellCenterToPosition(cit->row,cit->col,cellWidth,cellHeight);	//pasamos la celda actual a posicion
			cit->valor = (float) 1000/_distance(celdaActual,(*CentroDeExtraccion)->position);	// el valor sera 100 entre la distancia de la celda actual y el centro de extraccion
			cit++;//cambiamos de celda
	}
}

//Funcion de placeDefenses - En esta funcion crearemos el algoritmo devorador
void DEF_LIB_EXPORTED placeDefenses(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , std::list<Object*> obstacles, std::list<Defense*> defenses) {
	
    float cellWidth = mapWidth / nCellsWidth; //Ancho de celda
    float cellHeight = mapHeight / nCellsHeight; //Altura de celda
    int numDefensasColocadas = 0; //Numero de defensas colocadas

	//Definicion de variables que usaremos mas adelante:
	//Para la lista de candidatos usaremos una lista la cual almacenara las celdas de manera ordenada conforme al valor de la celda
	std::list<Celda> Celdas;
	//Definimos una celda auxiliar para poder seleccionar un candidato de la lista
	Celda auxiliar(0,0,0);
	//definimos un iterador de la lista de defensas para poder colocar las defensas
	List<Defense*>::iterator currentDefense = defenses.begin();
	
	//Inicializamos la lista de candidatos para colocar el centro de extraccion
	for(int i = 0 ; i < nCellsWidth ; i++)
	{
		for(int j = 0 ; j < nCellsHeight ; j++)
		{
			//insertamos todas las celdas en la lista de candidatos. Para poder ordenarla luego usaremos el metodo sort
			Celdas.push_front(Celda(i,j,cellValueExtrationCenter(i,j,freeCells,cellWidth,cellHeight,nCellsWidth,nCellsHeight)));
		}
	}
	Celdas.sort(); //Se realiza la ordenacion (al final de las listas estan las mejores celdas
	

	//Comenzaremos colocando el centro de extraccion de minerales (primera posicion de defensas)
	bool CentroColocado = false; //Definimos una variable que nos indicara true cuando el centro de extraccion sea colocado
	while(!Celdas.empty() && !CentroColocado) // Mientras que hayan candidatos(celdas)disponibles y el centro no haya sido colocado
	{
		auxiliar = Celdas.back(); //Guardamos el mejor candidato posible
		Celdas.pop_back(); //lo eliminamos de la lista de candidatos
		if(EsFactible(auxiliar.row,auxiliar.col,currentDefense,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses,numDefensasColocadas)) //Si la celda elegida es factible 
		{
			(*currentDefense)->position = cellCenterToPosition(auxiliar.row,auxiliar.col,cellWidth,cellHeight); //le damos a la primera defensa (Centro de Extraccion) la posicion de la celda elegida
			CentroColocado = true; //El centro ha sido colocado
			numDefensasColocadas++; //numero de defensas es 1
		}
	}
	//Actualizamos el valor de las celdas para que sea el mejor valor para las celdas
	cellValueDefenses(Celdas,currentDefense,cellWidth,cellHeight); 
	Celdas.sort();//Se realiza la ordenacion (al final de las listas estan las mejores celdas
	
	currentDefense++; //pasamos a la siguiente defensa

	//Continuamos con el colocado de las defensas restantes
	int maxAttemps = 1000;//establecemos un numero maximo de intentos
    while(currentDefense != defenses.end() && !Celdas.empty() && maxAttemps > 0) //Mientras que haya defensas por colocar y haya candidatos disponibles y el numero de intentos sea superior a 0
    {	 
		auxiliar = Celdas.back(); //seleccionamos el mejor candidato disponible
		Celdas.pop_back();//lo eliminamos de la lista de candidatos
		
    	if(EsFactible(auxiliar.row,auxiliar.col,currentDefense,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses,numDefensasColocadas)) //Si la celda elegida es factible
    	{
        	(*currentDefense)->position = cellCenterToPosition(auxiliar.row,auxiliar.col,cellWidth,cellHeight); //asignamos a la defensa la posicion de la celda elegida
			currentDefense++; //pasamos a la siguiente defensa
			numDefensasColocadas++; //defensa colocada +1
    	}
		maxAttemps--; //restamos 1 al numero de intentos
    }

#ifdef PRINT_DEFENSE_STRATEGY

    float** cellValues = new float* [nCellsHeight]; 
    for(int i = 0; i < nCellsHeight; ++i) {
       cellValues[i] = new float[nCellsWidth];
       for(int j = 0; j < nCellsWidth; ++j) {//DUPE
           cellValues[i][j] = ((int)(cellValue(i, j))) % 256;
       }
    }
    dPrintMap("strategy.ppm", nCellsHeight, nCellsWidth, cellHeight, cellWidth, freeCells
                         , cellValues, std::list<Defense*>(), true);

    for(int i = 0; i < nCellsHeight ; ++i)
        delete [] cellValues[i];
	delete [] cellValues;
	cellValues = NULL;

#endif
}

bool EsFactible(int row, int col,List<Defense*>::iterator currentdefense, int nCellsWidth, int nCellsHeight
	, float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses,int numDefensasColocadas)
{
	bool factible = true; //definimos una variable factible, la predefinimos con true, y si encontramos que no es factible la pondremos a false
	
	if(row >= 0 && row < nCellsWidth && col >= 0 && col < nCellsHeight) //si la row y col de la celda pasada esta dentro de rango
    {
		//definicon de variables necesarias para la verificacion de la factibilidad
		Vector3 pos = cellCenterToPosition(row,col,mapWidth/nCellsWidth,mapHeight/nCellsHeight); //obtenemos la posicion de la celda
		List<Object*>::iterator poit = obstacles.begin(); //definimos un iterador para la lista de obstulos inicializado con la primera posicion
		List<Defense*>::iterator pdit = defenses.begin(); //definimos un iterador para la lista de defensas inicializado con la primera posicion

		//Comprobamos que los obstaculos no chocan con nuestra defensa
		while(poit != obstacles.end() && factible)	//mientras que haya obstaculos por comprobar y factible sea verdadero
		{
			if(((*currentdefense)->radio + (*poit)->radio) > _distance(pos,(*poit)->position)) //si la suma de los radios de la defensa a colocar y el radio del obstaculo es mayor que la distancia desde el centro de la posicion de la celda hasta el centro de la posicion del obstaculo
				factible = false; //entonces la defensa no es colocable ya que estaria intersectando el radio del obstaculo (factible es falso)
			poit++; // pasamos al siguiente obstaculo
		}
		
		//Comprobamos que las defensas YA COLOCADAS no chocan con nuestra defensa
		while(pdit != defenses.end() && factible) //mientras que haya defensas por comprobar y factible sea verdadero
		{
			if(pdit == currentdefense || numDefensasColocadas == 0) //si la defensa que iteramos es la que vamos a colocar o numero de defensas colocadas es 0 (No hay defensa que pueda colisionar
			{
				pdit++; //pasamos a la siguiente defensa
				continue; //terminamos la iteracion actual en el bucle
			}
			else
			{
				if( ((*currentdefense)->radio + (*pdit)->radio) > _distance(pos,(*pdit)->position)) //si la suma del radios de la defensa a colocar y el radio de la defensa ya colocada es mayor que la distancia desde el centro de la posicion de la celda hasta el centro de la posicion defensa
					factible = false; //entonces la defensa no es colocable ya que estaria intersectando el radio de la otra defensa (factible es falso)
				pdit++; // pasamos a la siguiente defensa
				numDefensasColocadas--; //hemos comprobado esta defensa, por lo que ya no la contamos
			}
		}
		if(pos.x - (*currentdefense)->radio < 0 || pos.y - (*currentdefense)->radio < 0 ||//si la resta de la posicion x/y de la celda menos el radio de la defensa es menor que 0 o la suma de la posicion x/y de la celda mas el radio de la defensa
		pos.x + (*currentdefense)->radio > mapWidth || pos.y + (*currentdefense)->radio > mapHeight)// es mayor que el ancho/altura del mapa entonces el radio de la defensa ocupa una parte inexistente del borde del mapa
			factible = false; //entonces la defensa no es colocable ya que estaria violando los bordes del mapa (factible es falso)
	}
	else	//Si row y col esta fuera de rango
		factible = false; //entonces la defensa no es colocable (factible es falso)

	return factible; //devolvemos la factible
}

