#include <iostream>
#include <RRT.h>
#include <eigen3/Eigen/Dense>

#include <stdlib.h>

// USE STEPS

/*

1st: Create RRT object with an initial pose/wp

eg. RRT rrtObject(initial wp);

2nd: Given a proposed wp Check if it's a valid one

  *"Valid" means that this point is new and not similar to others.
  *The proposed wp must have been checked for obstacle presence previously.
  If obstacle presence

  rrtObject.AddObstacleChild(wp_i)

  If obstacle free, theen we run this method:

eg. rrtObject.AddNodeToCurrent(wp_i);

  * It will return 1 if it's a valid point.
  * If so. the drone MUST move to this wp because it's already in the tree
  * In other words, we first add it, inmediatly after we move to wp.

  else
  *It will return 0
  *If so. The wp is added to a blacklist, we must propose another one.
  *If the possibilities are emptied, then move back to parent and propose
  another one child.

  rrtObject.MoveBackward()

  REPEAT UNTIL GOAL REACHED




*/


int main(int argc, char** argv) {








  geometry_msgs::Point init;
  init.x = 0;
  init.y = 0;
  init.z = 0;

  RRT uno(init);

  std::cout << '\n' <<"TRIALS " << uno.getNumberOfTrials() << '\n';

  if(uno.AddNodeToCurrent(PI/2)){
    std::cout << "SE PUEDE AÑADIR" << '\n';
  }else{
    std::cout << "NO SE PUEDE AÑADIR" << '\n';
  }
  std::cout << '\n' <<"TRIALS " << uno.getNumberOfTrials() << '\n';

  uno.MoveBackward();

  std::cout << '\n' <<"TRIALS " << uno.getNumberOfTrials() << '\n';

  uno.AddObstacleChild(PI/2+1.7);
  std::cout << '\n' <<"TRIALS " << uno.getNumberOfTrials() << '\n';

  if(uno.AddNodeToCurrent(PI/2-1.7)){
    std::cout << "SE PUEDE AÑADIR" << '\n';
  }else{
    std::cout << "NO SE PUEDE AÑADIR" << '\n';
  }

  std::cout << '\n' <<"TRIALS " << uno.getNumberOfTrials() << '\n';

  uno.MoveBackward();

  std::cout << '\n' <<"TRIALS " << uno.getNumberOfTrials() << '\n';

  if(uno.AddNodeToCurrent(PI/2-1.7)){
    std::cout << "SE PUEDE AÑADIR" << '\n';
  }else{
    std::cout << "NO SE PUEDE AÑADIR" << '\n';
  }

  std::cout << '\n' <<"TRIALS " << uno.getNumberOfTrials() << '\n';

  return 0;
}
