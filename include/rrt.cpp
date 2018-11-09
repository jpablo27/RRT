#include <iostream>
#include <RRT.h>
#include <eigen3/Eigen/Dense>


RRT::RRT(geometry_msgs::Point initwp){
  angCompTh = 2*asin(dwp_th/(2*step));
  std::cout << "Constructor RRT" << std::endl;
  root = NULL;
  current = NULL;
  temp = NULL;

  RRT::AddNodeToCurrentPrivate(initwp);
}


RRT::~RRT(){
  std::cout << "Deconstructor RRT" << std::endl;
  RemoveRRT(root);

  std::cout << "------------------------" << '\n';

  for(int i=0; i<bestPath.size(); i++){
    std::cout << bestPath[i] << '\n';
  }

  std::cout << "------------------------" << '\n';
}

void RRT::RemoveRRT(node *Ptr){
  if (Ptr != NULL) {
    /* Actually pointing to a node*/
    int size=Ptr->childs.size();

    for(int i=0; i<size; i++){
      RemoveRRT(Ptr->childs[i]);
    }
    std::cout << "Now Deleting: " << Ptr->wp <<" Parent: "<< Ptr->parent<<'\n';

    delete Ptr;

  }else{
    /*There was nothing to delete*/
    delete root;
    delete current;
    delete temp;
  }
}

int RRT::AddNodeToCurrent(float alpha){


  wp_n = RRT::generatePoint(RRT::getCurrentPrivate(),alpha);

  if(RRT::CompareWp(wp_n, alpha)){
    std::cout << "CAN'T ACCEPT THIS POINT" << '\n';
    std::cout << "xxxxxxxxxxxxxxx" << '\n';
    std::cout << wp_n << '\n';
    std::cout << "xxxxxxxxxxxxxxx" << '\n'<< '\n';
    return 0;
  }else{
    std::cout << "ACCEPTED POINT" << '\n';

    RRT::AddNodeToCurrentPrivate(wp_n);
    return 1;
  }
}

void RRT::AddNodeToCurrentPrivate(geometry_msgs::Point wp){
  if(root == NULL){//Initialize with a first node, so wp needs to be the starting state.
    //std::cout << "Añadiendo Primer nodo: " << wp <<'\n';
    root = CreateLeaf(wp);
    root->parent = NULL;
    current = root;
    std::cout << "INITIALIZING WITH: "<< '\n'<< wp << '\n';
  }else{
    temp = CreateLeaf(wp);
    temp->parent = current;
    current->childs.push_back(temp);
    RRT::MoveForward();
  }
}


RRT::node* RRT::CreateLeaf(geometry_msgs::Point wp){
  node* n = new node;
  n->wp = wp;
  //std::cout << "Añadiendo nodo: " << wp <<" padre: "<<current << " Este: "<< n  <<'\n';
  bestPath.push_back(wp);
  return n;
}

void RRT::MoveForward(void){
  current = current->childs.back();
  std::cout << "Moving forward to: " << current->wp << '\n';
}

void RRT::MoveBackward(void){
  if(current->parent == NULL){
    std::cout << "You've reached the root node" << '\n';
  }else{
    current = current->parent;
    std::cout << "Moving backward to: " << current->wp << '\n';
    bestPath.pop_back();
  }
}

int RRT::CompareWp(geometry_msgs::Point wp, float alpha){

  if(current==NULL){
    std::cout << "1er punto, accepted" << '\n';
    return 0;
  }else if((current->parent) == NULL){
    std::cout << "Point from root, comparing to brothers" << '\n';
    for (int i = 0; i < (current->childs.size()); i++) {
      d2 = sqrt(pow((wp.x-(current->childs[i]->wp.x)),2)+pow((wp.y-(current->childs[i]->wp.y)),2));
      if(d2<=dwp_th) return 1;//wp's are similar, REJECT WP
    }
    //CHECK THE blacklist
    for (int i = 0; i < (current->headingBlackList.size()); i++) {
      angComp = alpha - current->headingBlackList[i];
      angComp = atan2(sin(angComp),cos(angComp));
      if((angComp<angCompTh)&&(angComp>-angCompTh)) return 1;//wp's in blacklist
    }

    return 0;
  }

  //CHECK THE blacklist
  for (int i = 0; i < (current->headingBlackList.size()); i++) {
    angComp = alpha - current->headingBlackList[i];
    angComp = atan2(sin(angComp),cos(angComp));
    if((angComp<angCompTh)&&(angComp>-angCompTh)) return 1;//wp's in blacklist
  }

  d1 = sqrt(pow((wp.x-(current->parent->wp.x)),2)+pow((wp.y-(current->parent->wp.y)),2));

  if(d1<=dwp_th) return 1;//wp's are similar, REJECT WP
  std::cout << "debugging" << '\n';
  for (int i = 0; i < (current->childs.size()); i++) {
    d2 = sqrt(pow((wp.x-(current->childs[i]->wp.x)),2)+pow((wp.y-(current->childs[i]->wp.y)),2));
    if(d2<=dwp_th) return 1;//wp's are similar, REJECT WP
  }

  return 0; //NEW valid wp, ACCEPT WP
}

geometry_msgs::Point RRT::generatePoint(geometry_msgs::Point basept,float alpha){
  geometry_msgs::Point pt;
  pt.x = basept.x + cos(alpha);
  pt.y = basept.y + sin(alpha);
  return pt;
}

geometry_msgs::Point RRT::getCurrentPrivate(void){
  return current->wp;
}

geometry_msgs::Point RRT::getCurrent(void){
  return getCurrentPrivate();
}

void RRT::AddObstacleChild(float alpha){
  current->headingBlackList.push_back(alpha);
}

int RRT::getNumberOfTrials(void){
  return current->headingBlackList.size() + current->childs.size();
}
