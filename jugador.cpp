#include "../Comportamientos_Jugador/jugador.hpp"
#include <cmath>
#include <set>
#include <map>
using namespace std;

#define DEBUG_1 0
#define DEBUG_3 0
#define ONE_OBJ 1
#define MAX_G 100000

/***************** ESTRUCTURAS AUXILIARES ******************/

// Compare nodes based on their position on the map
struct comp_pos {
  bool operator()(node * n1, node * n2) const {
    return n1->pos < n2->pos;
  }
};

// Maps each node to its fScore
map <node*, int, comp_pos> f;

/***************** FUNCIONES AUXILIARES ******************/

namespace {

Orientation operator+(const Action& act, const Orientation& ori) {
  int move;

  if (act == actTURN_L)
    move = -1;
  else if (act == actTURN_R)
    move = 1;

  switch(act) {
    case actTURN_L:
    case actTURN_R:
      return static_cast<Orientation>((ori + move + 4) % 4);
  }

  return ori;
}

Orientation operator+(const Orientation& ori, const Action& act) {
  return act + ori;
}

int manhattan_distance(const estado& origen, const estado& destino) {
  return abs(origen.fila - destino.fila) + abs(origen.columna - destino.columna);
}

node * mknode(const estado& pos, const estado& destino, node * parent,
              const stack<Action>& act) {
  node* n = new node(parent, pos, manhattan_distance(pos, destino), act);
  return n;
}

template <typename T>
void delete_set(set<node*, T> l) {
  for (auto it = l.begin(); it != l.end(); ++it) {
    delete *it;
  }
}

void printn(node * n) {
  cout << "(" << n->pos.fila << "," << n->pos.columna << "): g="
       << n->g << ", h=" << n->h << ", f=" << f[n] << endl;
}

void reconstruct_path(stack<Action>& plan, node * goal) {
  node* current = goal;

  while (current) {
    stack<Action> partial_path = current->actions;
    while (!partial_path.empty()) {
      plan.push(partial_path.top());
      partial_path.pop();
    }
    current = current->parent;
  }
}

void AnularMatriz(TMapa& m){
	for (int i=0; i<m[0].size(); i++){
		for (int j=0; j<m.size(); j++){
			m[i][j]=0;
		}
	}
}

void clear_stack(stack<Action>& st) {
  while (!st.empty())
    st.pop();
}

}

/***************** IMPLEMENTACIÓN DE MÉTODOS  *******************/

node::node(node * parent, const estado& pos, int h, stack<Action> actions) {
  this->g = MAX_G;
  this->parent = parent;
  this->pos = pos;
  this->h = h;
  this->actions = actions;
}

void ComportamientoJugador::PintaPlan(stack<Action> plan) {
	while (!plan.empty()){
    Action act = plan.top();
		if (act == actFORWARD){
			cout << "A ";
		}
		else if (act == actTURN_R){
			cout << "D ";
		}
		else if (act == actTURN_L){
			cout << "I ";
		}
		else {
			cout << "- ";
		}
		plan.pop();
	}
	cout << endl;
}

void ComportamientoJugador::actualizaPos(estado& pos, Action next) {
  if (next == actFORWARD) {
    switch(pos.orientacion) {
      case NORTH:
        pos.fila--; break;
      case SOUTH:
        pos.fila++; break;
      case EAST:
        pos.columna++; break;
      case WEST:
        pos.columna--; break;
    }
  }
  pos.orientacion = next + pos.orientacion;
}

bool ComportamientoJugador::esTransitable(int fil, int col,
                                          TMapa mapa, bool PK) const {
  bool transitable;
  if (fil >= 0 && col >= 0 && fil < mapa[0].size() && col < mapa[0].size()) {
    transitable = mapa[fil][col] == 'S'
                       || mapa[fil][col] == 'T'
                       || mapa[fil][col] == 'K'
                       || (!PK && mapa[fil][col] == '?');

#if DEBUG_3 == 1
  cout << "La casilla (" << fil << "," << col << ") es transitable: "
      << (transitable ? "SI" : "NO") << endl;
#endif

  }

#if DEBUG_3 == 1
  else {
    cout << "La casilla (" << fil << "," << col << ") está fuera de los límites." << endl;
    transitable = false;
  }
#endif

  return transitable;
}

/**
 * A* algorithm
 */
bool ComportamientoJugador::pathFinding(const estado& origen,
                                        const estado& destino,
                                        stack<Action>& plan,
                                        TMapa mapa,
                                        int vida,
                                        bool PK,
                                        int& cost) {
  // Custom comparator for nodes
  struct comp_f {
    bool operator()(node * n1, node * n2) const {
      int diff = f[n1] - f[n2];
      if (n1->pos == n2->pos)
        return false;
      if (diff != 0)
        return diff < 0;
      return n1->pos < n2->pos;
    }
  };

  bool found = false;
  int cont = 0;
  set<node*, comp_f> open_set;
  set<node*, comp_pos> closed_set;
  node* neighbors[4];
  int next_cost[4];
  int extra = 0;
  stack<Action> act;

#if DEBUG_1 == 1
  cout << "Destino -> (" << destino.fila << "," << destino.columna << ")" << endl;
#endif

  // Start node
  node* current = mknode(origen, destino, NULL, act);
  current->g = 0;
  f[current] = current->h;
  open_set.insert(current);

  // Main loop
  while (!open_set.empty() && !found) {
    auto current_it = open_set.begin();
    current = *current_it;

#if DEBUG_1 == 1
    cout << "Open -> ";
    printn(current);
#endif

    if (current->h == 0) { // h is the distance to the goal
      found = true;
      break;
    }

    closed_set.insert(current);
    open_set.erase(current_it);

    if (current->g > vida)
      continue;

    // Evaluate neighbors
    int valid = 0;
    estado pos = {current->pos.fila, current->pos.columna, current->pos.orientacion};
    estado tmp = pos;

    // Move forward
    actualizaPos(tmp, actFORWARD);
    if (esTransitable(tmp.fila, tmp.columna, mapa, PK)) {
      if (mapa[tmp.fila][tmp.columna] == '?')
        extra = 1;
      act.push(actFORWARD);
      neighbors[valid] = mknode(tmp, destino, current, act);
      next_cost[valid++] = 1 + extra;
      clear_stack(act);
    }
    tmp = pos;
    extra = 0;

    // Move east
    actualizaPos(tmp, actTURN_R);
    actualizaPos(tmp, actFORWARD);
    if (esTransitable(tmp.fila, tmp.columna, mapa, PK)) {
      if (mapa[tmp.fila][tmp.columna] == '?')
        extra = 1;
      act.push(actTURN_R);
      act.push(actFORWARD);
      neighbors[valid] = mknode(tmp, destino, current, act);
      next_cost[valid++] = 2 + extra;
      clear_stack(act);
    }
    tmp = pos;
    extra = 0;

    // Move west
    actualizaPos(tmp, actTURN_L);
    actualizaPos(tmp, actFORWARD);
    if (esTransitable(tmp.fila, tmp.columna, mapa, PK)) {
      if (mapa[tmp.fila][tmp.columna] == '?')
        extra = 1;
      act.push(actTURN_L);
      act.push(actFORWARD);
      neighbors[valid] = mknode(tmp, destino, current, act);
      next_cost[valid++] = 2 + extra;
      clear_stack(act);
    }
    tmp = pos;
    extra = 0;

    // Move backwards (only useful for the first move)
    if (cont == 0) {
      actualizaPos(tmp, actTURN_R);
      actualizaPos(tmp, actTURN_R);
      actualizaPos(tmp, actFORWARD);
      if (esTransitable(tmp.fila, tmp.columna, mapa, PK)) {
        act.push(actTURN_R);
        act.push(actTURN_R);
        act.push(actFORWARD);
        neighbors[valid] = mknode(tmp, destino, current, act);
        next_cost[valid++] = 3;
        clear_stack(act);
      }
      tmp = pos;
    }

    for (int i = 0; i < valid; i++) {
      if (!closed_set.count(neighbors[i])) {
        auto it = open_set.find(neighbors[i]);
        bool found_in_open = it != open_set.end();
        int tentative_g = current->g + next_cost[i];
        bool better = !found_in_open || tentative_g < (*it)->g;

        if (better) {
          neighbors[i]->g = tentative_g;
          f[neighbors[i]] = neighbors[i]->g + neighbors[i]->h;
        }

#if DEBUG_1 == 1
      if (better) {
        cout << "Neighbor -> ";
        printn(neighbors[i]);
      }
#endif

        if (better) { // Update (or insert) node in open set
          if (found_in_open)
            open_set.erase(it);
          open_set.insert(neighbors[i]);
        }
        else {
          delete neighbors[i];
        }
      }
      else { // Already in closed set
        delete neighbors[i];
      }
    }
  cont++;
  }

  if (found) {
    cont++;  // Take into account the expansion of the goal node

#if DEBUG_1 == 1
    cout << "---------- PATH FOUND ----------\n" << endl;
    cout << "Nodos abiertos: " << cont << endl;
#endif

    // Generate actual path and print it
    reconstruct_path(plan, current);
    cost = current->g;
  }
  else {
    cost = -1;
  }

  // Free memory
  delete_set(open_set);
  delete_set(closed_set);
  f.clear();

  return found;
}

void ComportamientoJugador::actualizaMapa(TMapa& mapa,
                                          vector<unsigned char> terreno) {
  int cont = 0;
  int fil;
  int col;

  for (int i = 0; i <= 3; i++) {
    for (int j = -i; j <= i; j++) {
      switch(pos.orientacion) {
        case EAST:
          fil = pos.fila + j;
          col = pos.columna + i;
          break;
        case WEST:
          fil = pos.fila - j;
          col = pos.columna - i;
          break;
        case NORTH:
          fil = pos.fila - i;
          col = pos.columna + j;
          break;
        case SOUTH:
          fil = pos.fila + i;
          col = pos.columna - j;
          break;
      }

      if (mapa[fil][col] == '?') {
        mapa[fil][col] = terreno[cont];

        if (!bien_situado && terreno[cont] == 'K') {
          if (pk[0].fila == -1) {
            pk[0].fila = fil;
            pk[0].columna = col;
          }
          else {
            pk[1].fila = fil;
            pk[1].columna = col;
          }
        }
      }
      cont++;
    }
  }
}

void ComportamientoJugador::vuelcaMapaLocal() {
  int inicio_fil = pk[0].fila - pos.fila;
  int inicio_col = pk[0].columna - pos.columna;
  int size = mapaResultado[0].size();

  for (int i = 0; i < size; i++)
    for (int j = 0; j < size; j++)
      if (mapaLocal[i + inicio_fil][j + inicio_col] != '?')
        mapaResultado[i][j] = mapaLocal[i + inicio_fil][j + inicio_col];
}

void ComportamientoJugador::intentaCaminar(estado destino, TMapa mapa, int vida, bool PK) {
  if (!hay_plan) {
    if (!(hay_plan = pathFinding(pos, destino, plan, mapa, vida, PK, total_cost))) {
      if (!PK) {
        cout << "Error: no hay camino posible al destino." << endl;
        error = true;
      }
      else {  // No merece la pena ir si no hay camino posible ahora mismo
        pk[0].fila = pk[1].fila;
        pk[0].columna = pk[1].columna;
        pk[1].fila = -1;
        pk[1].columna = -1;
      }
    }
    else if (!PK) {
      VisualizaPlan(pos, plan);
    }
  }

  if (!plan.empty()) {
    Action next_move = plan.top();
    estado next = pos;
    actualizaPos(next, next_move);

    if (next_move == actFORWARD && !esTransitable(next.fila, next.columna, mapa, PK)) {

#if DEBUG_3 == 1
      cout << "No puedes pasar por la casilla que tienes delante." << endl;
#endif

      hay_plan = false;
      clear_stack(plan);
    }
  }
}

Action ComportamientoJugador::caminar(vector<unsigned char> superficie) {
  Action next_move;
  if ((!bien_situado || hay_plan) && !plan.empty()) {
    next_move = plan.top();
    if (next_move == actFORWARD && superficie[2] == 'a')
      next_move = actIDLE;
    else
      plan.pop();
  }
  else {
    next_move = actIDLE;
  }

  return next_move;
}

estado ComportamientoJugador::buscarPK() {
  int minimo = cont_calor;
  int fil, col;
  estado dest;

  for (int i = -1; i <=1; i++) {
    if (i == 0) {
      for (int j = -1; j <= 1; j += 2) {
        fil = pos.fila;
        col = pos.columna + j;
        if (esTransitable(fil, col, mapaLocal, true) && mapaCalor[fil][col] < minimo) {
          minimo = mapaCalor[fil][col];
          dest = estado{fil, col, NORTH};
        }
      }
    }
    else {
      fil = pos.fila + i;
      col = pos.columna;
      if (esTransitable(fil, col, mapaLocal, true) && mapaCalor[fil][col] < minimo) {
        minimo = mapaCalor[fil][col];
        dest = estado{fil, col, NORTH};
      }
    }
  }

  return dest;
}

Action ComportamientoJugador::think(Sensores sensores) {

#if ONE_OBJ == 1
  if (nivel3) {
    static bool one_obj = false;
    static estado destino_orig;
    static bool primera = true;

    if (primera) {
      destino_orig.fila = sensores.destinoF;
      destino_orig.columna = sensores.destinoC;
      primera = false;
    }

    if (sensores.destinoF != destino_orig.fila || sensores.destinoC != destino_orig.columna) {
      //cout << "Se ha encontrado un objetivo. Ya no se buscarán más." << endl;
      one_obj = true;
    }

    if (one_obj)
      return actTURN_R;
  }
#endif

  static int reconocimiento = 0;

  if (sensores.mensajeF != -1 && !bien_situado) {
    if (nivel3) {
      actualizaPos(pos, ultima_accion);
      pk[0].fila = pos.fila;
      pk[0].columna = pos.columna;
    }
    pos.fila = sensores.mensajeF;
    pos.columna = sensores.mensajeC;
    bien_situado = true;

    if (nivel3) {
      cout << "Pisada casilla K en (" << pk[0].fila << "," << pk[0].columna << ")\n";
      vuelcaMapaLocal();
      ultima_accion = actIDLE;
      clear_stack(plan);
    }
  }

  actualizaPos(pos, ultima_accion);


  if (bien_situado && (hay_plan || error)
      && (sensores.destinoF != destino.fila || sensores.destinoC != destino.columna)) {
    if (!nivel3)
      cout << "\nObjetivo cambiado." << endl;
    hay_plan = false;
    error = false;
    clear_stack(plan);
  }

  if (error)
    return actIDLE;

  if (!bien_situado) {  // NIVEL 3 SIN CONOCER MI POSICIÓN
    actualizaMapa(mapaLocal, sensores.terreno);

    // Andar hacia PK
    if (pk[0].fila != -1) {

#if DEBUG_3 == 1
      cout << "Localizada casilla K en (" << pk[0].fila << "," << pk[0].columna << ")\n";
#endif
      reconocimiento = 4;
      intentaCaminar(pk[0], mapaLocal, sensores.vida, true);
      ultima_accion = caminar(sensores.superficie);
      return ultima_accion;
    }

    // Bucar PK
    if (pk[0].fila == -1) {

      // Reconocer los alrededores la primera vez
      if (reconocimiento < 4) {
        reconocimiento++;
        ultima_accion = actTURN_R;
        return ultima_accion;
      }

      estado next_dest;

      if (plan.empty()) {
        mapaCalor[pos.fila][pos.columna] = cont_calor++;
        next_dest = buscarPK();
        pathFinding(pos, next_dest, plan, mapaLocal, sensores.vida, true, total_cost);
      }

      ultima_accion = caminar(sensores.superficie);
      return ultima_accion;
      }
    }

      /*
      estado next_s;

      do {
        int random = aleatorio(8);

        switch (random) {
          case 1: ultima_accion = actTURN_R; break;
          case 2: ultima_accion = actTURN_L; break;
          case 3: case 4: case 5: case 6: case 7: case 8: ultima_accion = actFORWARD;
        }

        if (ultima_accion == actFORWARD) {
          next_s = pos;
          actualizaPos(next_s, actFORWARD);
        }

      } while (ultima_accion == actFORWARD && !esTransitable(next_s.fila, next_s.columna, mapaLocal, true));

      if (ultima_accion == actFORWARD) {
        if (sensores.superficie[2] != 'a')
          ultima_accion = actFORWARD;
        else
          ultima_accion = actIDLE;
      }*/


  else if (nivel3) { // NIVEL 3
    actualizaMapa(mapaResultado, sensores.terreno);

    while (!hay_plan && !error) {
      destino.fila = sensores.destinoF;
      destino.columna = sensores.destinoC;

      intentaCaminar(destino, mapaResultado, sensores.vida, false);
    }

    if (hay_plan)
      intentaCaminar(destino, mapaResultado, sensores.vida, false);

    ultima_accion = caminar(sensores.superficie);
    return ultima_accion;
  }

  else {  // NIVEL 1 O 2
    if (!hay_plan) {
      destino.fila = sensores.destinoF;
      destino.columna = sensores.destinoC;
      if (!(hay_plan = pathFinding(pos, destino, plan, mapaResultado,
                                   sensores.vida, false, total_cost)))
        cout << "Error: no hay camino posible al destino." << endl;
      else {
        VisualizaPlan(pos, plan);
        cout << "Coste total del camino: " << total_cost << endl;
      }
    }

    ultima_accion = caminar(sensores.superficie);
    return ultima_accion;
  }
}

void ComportamientoJugador::VisualizaPlan(const estado &st, stack<Action> plan){
  AnularMatriz(mapaConPlan);
	estado cst = st;
	while (!plan.empty()){
    Action next = plan.top();
		actualizaPos(cst, next);
    mapaConPlan[cst.fila][cst.columna] = 1;
		plan.pop();
	}
}

int ComportamientoJugador::interact(Action accion, int valor) {
  return false;
}
