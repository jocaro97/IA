#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"
#include "motorlib/util.h"
#include <iostream>
#include <stack>
#include <list>

typedef vector<vector<unsigned char> > TMapa;
typedef vector<vector<unsigned int> > TMapaCalor;

// 0=Norte, 1=Este, 2=Sur, 3=Oeste
enum Orientation {NORTH, EAST, SOUTH, WEST};

struct estado {
  int fila;
  int columna;
  Orientation orientacion;

  estado(int fil, int col, Orientation ori = NORTH) {
    fila = fil;
    columna = col;
    orientacion = ori;
  }

  estado() : estado(-1, -1) { }

  bool operator<(const estado& otro) const {
    if (fila != otro.fila)
      return fila < otro.fila;
    return columna < otro.columna;
  }

  bool operator==(const estado& otro) const {
    return !(*this < otro || otro < *this);
  }

  bool operator !=(const estado& otro) const {
    return !(*this == otro);
  }
};

struct node {
  int h;
  int g;
  estado pos;
  stack<Action> actions; // Actions performed to get from parent to this node
  node* parent;

  node(node * parent, const estado& pos, int h, stack<Action> actions);
};

class ComportamientoJugador : public Comportamiento {
  public:
    void constructor(int size = 100) {
      // Inicializar Variables de Estado
      pos.fila = pos.columna = size - 1;
      pos.orientacion = NORTH; // Siempre aparecemos mirando al norte
      destino.fila = -1;
      destino.columna = -1;
      destino.orientacion = NORTH; // Indiferente
      ultima_accion = actIDLE;
      hay_plan = bien_situado = error = false;
      cont_calor = 1;
    }

    ComportamientoJugador(unsigned int size) : Comportamiento(size) {
      constructor(size);
      nivel3 = true;
      srand(time(NULL));

      vector<unsigned char> aux (2*size, '?');
      vector<unsigned int> aux2(2*size, 0);
      for (int i = 0; i < 2*size; i++) {
        mapaLocal.push_back(aux);
        mapaCalor.push_back(aux2);
      }
    }
    ComportamientoJugador(std::vector< std::vector< unsigned char> > mapaR) : Comportamiento(mapaR) {
      constructor();
      nivel3 = false;
    }
    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport) {}
    ~ComportamientoJugador() {}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);
    ComportamientoJugador * clone() {return new ComportamientoJugador(*this);}

  private:
    // Declarar Variables de Estado
    estado pos, destino;
    estado pk[2];
    bool hay_plan;
    bool bien_situado;
    bool nivel3;
    bool error;
    int total_cost; // Coste total del camino actual si hay plan
    int cont_calor;
    Action ultima_accion;
    stack<Action> plan;
    TMapa mapaLocal;
    TMapaCalor mapaCalor;

    void actualizaPos(estado& pos, Action next);
    void actualizaMapa(TMapa& mapa, vector<unsigned char> terreno);
    void vuelcaMapaLocal();
    Action caminar(vector<unsigned char> superficie);
    void intentaCaminar(estado destino, TMapa mapa, int vida, bool PK);
    estado buscarPK();
    bool esTransitable(int fil, int col, TMapa mapa, bool PK) const;
    bool pathFinding(const estado &origen, const estado &destino, stack<Action> &plan,
                     TMapa mapa, int vida, bool PK, int& cost);
    void VisualizaPlan(const estado &st, stack<Action> plan);
    void PintaPlan(stack<Action> plan);
};

#endif
