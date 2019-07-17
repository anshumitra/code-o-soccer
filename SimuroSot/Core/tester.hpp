# pragma once
#include "../mainheaders.hpp"
#include "../common/include/cs.hpp"
#include "../winDebugger/Client.h"
using namespace std;
using namespace MyStrategy;
bool FIELD_IS_INVERTED = false;
class Tester
{
public:
  BeliefState &state;
  
  Comm*    comm;
 
  Tester(BeliefState &state_):
	  state(state_)
  {
  }
  void run()
  {
    game(&state);

    return;
  }
};