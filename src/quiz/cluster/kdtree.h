/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}


	void insert_helper(Node** node, uint depth, int id, std::vector <float> point){

		// Base Case

		if (*node == NULL){
			*node = new Node(point, id); 
		}
		else {
			// Recursive Functionality
			if (point[depth % 3]  < ((*node) -> point[depth % 3])){
				insert_helper(&((*node) -> left), ++depth, id, point); 
			}
			else {
				insert_helper(&((*node) -> right), ++depth, id, point); 
		}

		}

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert_helper(&root, 0, id, point); 
		}


	void search_helper(std::vector<float> target, Node* starting_node, float distanceTol, 
	std::vector<int>& ids, uint depth = 0)
	{

		if (starting_node != NULL){
			// Less expensive distance checking
			if (starting_node -> point[0] >= target[0] - distanceTol
			&& starting_node -> point[0] <= target[0] + distanceTol
			&& starting_node -> point[1] >= target[1] - distanceTol
			&& starting_node -> point[1] <= target[1] + distanceTol
			&& starting_node -> point[2] >= target[2] - distanceTol
			&& starting_node -> point[2] <= target[2] + distanceTol
			){
				 //More expensive distance check
				 float dis = sqrtf(pow(target[0] - starting_node -> point[0], 2) + pow(target[1] - starting_node -> point[1], 2) + \
				 pow(target[2] - starting_node -> point[2], 2));  
				 
				 if (dis <= distanceTol){
					 ids.push_back(starting_node -> id); 
					 }
			}


			if (target[depth % 3] - distanceTol < starting_node -> point[depth % 3])
			{
				// if lowest tolerance < point -> look into low tolerant area
				search_helper(target, starting_node -> left, distanceTol, ids, ++depth); 
			}
			if (distanceTol + target[depth % 3] > starting_node -> point[depth % 3])
			{
				// if highest tolerance > point -> look in high tolerant area
				search_helper(target, starting_node -> right, distanceTol, ids, ++depth); 
			}

	}

	}
	
		
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		
		std::vector<int> ids; 
		search_helper(target, root, distanceTol, ids); 
		return ids;
	}
	

};



