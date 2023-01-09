// ###### Config options ################

//P3

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"
#include "cronometro.h"

using namespace Asedio;     
// <------------------------------------------------------------------ CABECERAS DE LA P3 ------------------------------------------------------------------>

void greedyWithoutPreor(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
			, std::list<Object*> obstacles, std::list<Defense*> defenses);

void greedyWhithOrFus(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
			, std::list<Object*> obstacles, std::list<Defense*> defenses);

void greedyWhithOrQuick(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
			, std::list<Object*> obstacles, std::list<Defense*> defenses);
void greedyWhithHeap(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
			, std::list<Object*> obstacles, std::list<Defense*> defenses);


//	<------------------------------------------------------------------ AÑADIDO DE LA P1 ------------------------------------------------------------------>



//Estructura de Celda, formada por una fila, columna y un valor (cuanto de centrada es la celda)
struct Celda
{
	int row,col;
	float valor;
	Celda(int r, int c, int v): row(r), col(c), valor(v) {}
	Celda(): row(0),col(0),valor(0) {}
};

//operador de menor para hacer poder ordenar las celdas en la lista en referencia a su valor
bool operator<(const Celda& c1,const Celda& c2)
{
	return c1.valor < c2.valor;
} 

bool operator<=(const Celda& c1,const Celda& c2)
{
	return c1.valor <= c2.valor;
} 

//Funciones proporcionadas:
//Pasar de celda a posicion y de posicion a celda
Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight){ return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0); }
void positionToCell(const Vector3 pos, int &i_out, int &j_out, float cellWidth, float cellHeight){ i_out = (int)(pos.y * 1.0f/cellHeight); j_out = (int)(pos.x * 1.0f/cellWidth); }

//Funcion de factibilidad
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



//	^------------------------------------------------------------------ AÑADIDO DE LA P1 ------------------------------------------------------------------^


    

float defaultCellValue(int row, int col, bool** freeCells, int nCellsWidth, int nCellsHeight               
    , float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses) {
    	
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;

    Vector3 cellPosition((col * cellWidth) + cellWidth * 0.5f, (row * cellHeight) + cellHeight * 0.5f, 0);
    	
    float val = 0;
    for (List<Object*>::iterator it=obstacles.begin(); it != obstacles.end(); ++it) {
	    val += _distance(cellPosition, (*it)->position);
    }

    return val;
}

void DEF_LIB_EXPORTED placeDefenses3(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , List<Object*> obstacles, List<Defense*> defenses) {

    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight; 
	double t1,t2,t3,t4;

	//cronometro c;
	
	//c.activar();
	//greedyWithoutPreor(freeCells,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses);
	//c.parar();
	
	//t1 = c.tiempo();
	
	//c.activar();
	//greedyWhithOrFus(freeCells,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses);
	//c.parar();

	//t2 = c.tiempo();
	
	//c.activar();
	//greedyWhithOrQuick(freeCells,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses);
	//c.parar();

	//t3 = c.tiempo();
	
	//c.activar();
	greedyWhithHeap(freeCells,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses);
	//c.parar();
	
	//t4 = c.tiempo();

    //std::cout << t1 << " " << t2 << " " << t3 << " " << t4 <<"\n";
}
//<------------------------------------------------------------------ Sin preordenacion ------------------------------------------------------------------>


std::vector<Celda>::iterator bestCandidate(std::vector<Celda>& Celdas)
{
	std::vector<Celda>::iterator currentCell = Celdas.begin(),i;
	Celda Max(0,0,0);
	
	while(currentCell != Celdas.end())
	{
		if(Max < (*currentCell))
		{
			Max = (*currentCell);
			i = currentCell;
		}
		currentCell++;
	}
	
	return i;
}

void greedyWithoutPreor(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
			, std::list<Object*> obstacles, std::list<Defense*> defenses) 
{
    float cellWidth = mapWidth / nCellsWidth; //Ancho de celda
    float cellHeight = mapHeight / nCellsHeight; //Altura de celda
    int numDefensasColocadas = 0; //Numero de defensas colocadas

	//Definicion de variables que usaremos mas adelante:
	//Para la lista de candidatos usaremos una lista la cual almacenara las celdas de manera ordenada conforme al valor de la celda
	std::vector<Celda> Celdas;
	//Definimos una celda auxiliar para poder seleccionar un candidato de la lista
	Celda auxiliar(0,0,0);
	std::vector<Celda>::iterator aux;
	//definimos un iterador de la lista de defensas para poder colocar las defensas
	List<Defense*>::iterator currentDefense = defenses.begin();
	
	//Inicializamos la lista de candidatos para colocar el centro de extraccion
	for(int i = 0 ; i < nCellsWidth ; i++)
	{
		for(int j = 0 ; j < nCellsHeight ; j++)
		{
			//insertamos todas las celdas en la lista de candidatos. Para poder ordenarla luego usaremos el metodo sort
			Celdas.push_back(Celda(i,j,defaultCellValue(i,j,freeCells,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses)));
		}
	}
	
	
	//Continuamos con el colocado de las defensas restantes
	int maxAttemps = 1000;//establecemos un numero maximo de intentos
    while(currentDefense != defenses.end() && !Celdas.empty() && maxAttemps > 0) //Mientras que haya defensas por colocar y haya candidatos disponibles y el numero de intentos sea superior a 0
    {	 
		aux = bestCandidate(Celdas);
		auxiliar = (*aux);
		Celdas.erase(aux);

    	if(EsFactible(auxiliar.row,auxiliar.col,currentDefense,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses,numDefensasColocadas)) //Si la celda elegida es factible
    	{
        	(*currentDefense)->position = cellCenterToPosition(auxiliar.row,auxiliar.col,cellWidth,cellHeight); //asignamos a la defensa la posicion de la celda elegida
			currentDefense++; //pasamos a la siguiente defensa
			numDefensasColocadas++; //defensa colocada +1
    	}
		maxAttemps--; //restamos 1 al numero de intentos
		
    }
}

//^------------------------------------------------------------------ Sin preordenacion ------------------------------------------------------------------^


//<------------------------------------------------------------------ Con preordenacion de Fusion ------------------------------------------------------------------>
std::vector<Celda> fusion(std::vector<Celda>& FirstHalf,std::vector<Celda>& SecondHalf){

	std::vector<Celda> Results(FirstHalf.size() + SecondHalf.size());// Vector resultado (Vector ordenado creado por la fusion entre la primera mitad y la segunda)
	int i = 0, j = 0, k = 0;//Variables necesarias para recorrer cada vector
	
	
	
	while(i < FirstHalf.size() && j < SecondHalf.size()) //Mientras que no hayamos recorrido uno de los vectores enteros
	{
		if(FirstHalf[i] < SecondHalf[j]) //si la posicion i de la primera mitad es menor que la posicion j de la segunda mitad
		{
			Results[k] = FirstHalf[i];	//Asignamos el elemento en resultado
			i++;//iteramos i para la primera mitad
		}
		else
		{
			Results[k] = SecondHalf[j];//Asignamos el elemento en resultado
			j++; //iteramos j para la segunda mitad
		}
		k++; //iteramos k para el resultado
	}
	//Rellenar lo faltante de cada mitad
	while(i < FirstHalf.size())
	{
		Results[k] = FirstHalf[i];
		i++;
		k++;
	}
	while(j < SecondHalf.size())
	{
		Results[k] = SecondHalf[j];
		j++;
		k++;
	}
	return Results;
}


void ordenacionFusion(std::vector<Celda>& v)
{
    int n = v.size()/2; //obtenemos la posicion de la mitad del vector
    
    if(n > 0)	//si es mayor de 0
    {
    	std::vector<Celda> FirstHalf(n); //creamos 2 vectores auxiliares
    	std::vector<Celda> SecondHalf(v.size() - n);// Uno guardara la primera mitad y otro la segunda
    	
    	for(int i = 0; i < n; i++)	//relleno de la primera mitad
    	{
    		FirstHalf[i] = v[i];
    	}
    	
    	for(int i = n ; i < v.size(); i++) //relleno de la segunda mitad
    	{
    		SecondHalf[i-n] = v[i];
    	}
    	ordenacionFusion(FirstHalf); //llamada recursiva con la primera mitad
    	ordenacionFusion(SecondHalf);//llamada recursiva con la segunda mitad
    	v = fusion(FirstHalf,SecondHalf); //llamada a funsion
    }
    
}


void greedyWhithOrFus(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
			, std::list<Object*> obstacles, std::list<Defense*> defenses) 
{
	    float cellWidth = mapWidth / nCellsWidth; //Ancho de celda
    float cellHeight = mapHeight / nCellsHeight; //Altura de celda
    int numDefensasColocadas = 0; //Numero de defensas colocadas

	//Definicion de variables que usaremos mas adelante:
	//Para la lista de candidatos usaremos una lista la cual almacenara las celdas de manera ordenada conforme al valor de la celda
	std::vector<Celda> Celdas;
	//Definimos una celda auxiliar para poder seleccionar un candidato de la lista
	Celda auxiliar(0,0,0);
	std::vector<Celda>::iterator aux;
	//definimos un iterador de la lista de defensas para poder colocar las defensas
	List<Defense*>::iterator currentDefense = defenses.begin();
	
	//Inicializamos la lista de candidatos para colocar el centro de extraccion
	for(int i = 0 ; i < nCellsWidth ; i++)
	{
		for(int j = 0 ; j < nCellsHeight ; j++)
		{
			//insertamos todas las celdas en la lista de candidatos. Para poder ordenarla luego usaremos el metodo sort
			Celdas.push_back(Celda(i,j,defaultCellValue(i,j,freeCells,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses)));
		}
	}
	
	//Llamada a la funcion fusion para la ordenacion
	ordenacionFusion(Celdas);
	
	
	//Continuamos con el colocado de las defensas restantes
	int maxAttemps = 1000;//establecemos un numero maximo de intentos
    while(currentDefense != defenses.end() && !Celdas.empty() && maxAttemps > 0) //Mientras que haya defensas por colocar y haya candidatos disponibles y el numero de intentos sea superior a 0
    {	 
		aux = (Celdas.end() - 1);
		auxiliar = (*aux);
		Celdas.erase(aux);
		

    	if(EsFactible(auxiliar.row,auxiliar.col,currentDefense,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses,numDefensasColocadas)) //Si la celda elegida es factible
    	{
        	(*currentDefense)->position = cellCenterToPosition(auxiliar.row,auxiliar.col,cellWidth,cellHeight); //asignamos a la defensa la posicion de la celda elegida
			currentDefense++; //pasamos a la siguiente defensa
			numDefensasColocadas++; //defensa colocada +1
    	}
		maxAttemps--; //restamos 1 al numero de intentos
		
    }
   
}

//^------------------------------------------------------------------ Con preordenacion de Fusion ------------------------------------------------------------------^

//<------------------------------------------------------------------ Con preordenacion de Ordenacion Rapida ------------------------------------------------------------------>

int pivote(std::vector<Celda>& v,int i, int j)
{
	int pivot = i;
	int x = v[i].valor;
	
	for(int k = i +1 ; k <= j ; k++)
	{
		if(v[k].valor <= x)
		{
			pivot++;
			Celda aux = v[k];
			v[k] = v[pivot];
			v[pivot] = aux;
		}
	}
	
	v[i] = v[pivot];
	v[pivot].valor = x;
	return pivot;
}

void Quicksort(std::vector<Celda>& v,int i, int j)
{
	int n = j-i+1; //obtenemos el numero de elementos del rango del vector
	
	if(n > 0)	//si es mayor de 0
	{
		int pivot = pivote(v,i,j);
		Quicksort(v,i,pivot-1);
		Quicksort(v,pivot+1,j);
	}
}

void greedyWhithOrQuick(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
			, std::list<Object*> obstacles, std::list<Defense*> defenses)
{

	float cellWidth = mapWidth / nCellsWidth; //Ancho de celda
    float cellHeight = mapHeight / nCellsHeight; //Altura de celda
    int numDefensasColocadas = 0; //Numero de defensas colocadas

	//Definicion de variables que usaremos mas adelante:
	//Para la lista de candidatos usaremos una lista la cual almacenara las celdas de manera ordenada conforme al valor de la celda
	std::vector<Celda> Celdas;
	//Definimos una celda auxiliar para poder seleccionar un candidato de la lista
	Celda auxiliar(0,0,0);
	std::vector<Celda>::iterator aux;
	//definimos un iterador de la lista de defensas para poder colocar las defensas
	List<Defense*>::iterator currentDefense = defenses.begin();
	
	//Inicializamos la lista de candidatos para colocar el centro de extraccion
	for(int i = 0 ; i < nCellsWidth ; i++)
	{
		for(int j = 0 ; j < nCellsHeight ; j++)
		{
			//insertamos todas las celdas en la lista de candidatos. Para poder ordenarla luego usaremos el metodo sort
			Celdas.push_back(Celda(i,j,defaultCellValue(i,j,freeCells,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses)));
		}
	}
	
	//Llamada a la funcion Quicksort para la ordenacion
	Quicksort(Celdas,0,Celdas.size()-1);
	
	//Continuamos con el colocado de las defensas restantes
	int maxAttemps = 1000;//establecemos un numero maximo de intentos
    while(currentDefense != defenses.end() && !Celdas.empty() && maxAttemps > 0) //Mientras que haya defensas por colocar y haya candidatos disponibles y el numero de intentos sea superior a 0
    {	 
		aux = (Celdas.end() - 1);
		auxiliar = (*aux);
		Celdas.erase(aux);
		

    	if(EsFactible(auxiliar.row,auxiliar.col,currentDefense,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses,numDefensasColocadas)) //Si la celda elegida es factible
    	{
        	(*currentDefense)->position = cellCenterToPosition(auxiliar.row,auxiliar.col,cellWidth,cellHeight); //asignamos a la defensa la posicion de la celda elegida
			currentDefense++; //pasamos a la siguiente defensa
			numDefensasColocadas++; //defensa colocada +1
    	}
		maxAttemps--; //restamos 1 al numero de intentos
		
    }
}
//^------------------------------------------------------------------ Con preordenacion de Ordenacion Rapida ------------------------------------------------------------------^

//<------------------------------------------------------------------ Con Monticulo ------------------------------------------------------------------>

void greedyWhithHeap(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
			, std::list<Object*> obstacles, std::list<Defense*> defenses)
{
	float cellWidth = mapWidth / nCellsWidth; //Ancho de celda
    float cellHeight = mapHeight / nCellsHeight; //Altura de celda
    int numDefensasColocadas = 0; //Numero de defensas colocadas

	//Definicion de variables que usaremos mas adelante:
	//Para la lista de candidatos usaremos una lista la cual almacenara las celdas de manera ordenada conforme al valor de la celda
	std::vector<Celda> Celdas;
	//Definimos una celda auxiliar para poder seleccionar un candidato de la lista
	Celda auxiliar(0,0,0);
	std::vector<Celda>::iterator aux;
	//definimos un iterador de la lista de defensas para poder colocar las defensas
	List<Defense*>::iterator currentDefense = defenses.begin();
	
	//Inicializamos la lista de candidatos para colocar el centro de extraccion
	for(int i = 0 ; i < nCellsWidth ; i++)
	{
		for(int j = 0 ; j < nCellsHeight ; j++)
		{
			//insertamos todas las celdas en la lista de candidatos. Para poder ordenarla luego usaremos el metodo sort
			Celdas.push_back(Celda(i,j,defaultCellValue(i,j,freeCells,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses)));
		}
	}
	
	//Llamada a la funcion Quicksort para la ordenacion
	std::make_heap(Celdas.begin(),Celdas.end());
	
	//Continuamos con el colocado de las defensas restantes
	int maxAttemps = 1000;//establecemos un numero maximo de intentos
    while(currentDefense != defenses.end() && !Celdas.empty() && maxAttemps > 0) //Mientras que haya defensas por colocar y haya candidatos disponibles y el numero de intentos sea superior a 0
    {	 
		auxiliar = Celdas.front();
		std::pop_heap(Celdas.begin(),Celdas.end()); 
		Celdas.pop_back();		

    	if(EsFactible(auxiliar.row,auxiliar.col,currentDefense,nCellsWidth,nCellsHeight,mapWidth,mapHeight,obstacles,defenses,numDefensasColocadas)) //Si la celda elegida es factible
    	{
        	(*currentDefense)->position = cellCenterToPosition(auxiliar.row,auxiliar.col,cellWidth,cellHeight); //asignamos a la defensa la posicion de la celda elegida
			currentDefense++; //pasamos a la siguiente defensa
			numDefensasColocadas++; //defensa colocada +1
    	}
		maxAttemps--; //restamos 1 al numero de intentos
    }
}
//^------------------------------------------------------------------ Con Monticulo ------------------------------------------------------------------^

void CajaNegraFusion(std::vector<Celda>& v)
{
	ordenacionFusion(v); //realizamos ordenacion
	bool bienordenado = true;
	for(int i = 0; i < v.size()-1 ; i++)
	{
		if(v[i+1] < v[i])
		{
			bienordenado = false;
		}
	}
}

void CajaNegraQuickSort(std::vector<Celda>& v)
{
	Quicksort(v,0,v.size()-1);
	bool bienordenado = true;
	for(int i = 0; i < v.size()-1 ; i++)
	{
		if(v[i+1] < v[i])
		{
			bienordenado = false;
		}
	}
}
