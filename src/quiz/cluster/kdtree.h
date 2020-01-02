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

    void insertHelper(Node* &node, std::vector<float> point, uint depth,  int id)
    {
        if(node == NULL)
        {
            node = new Node(point, id);
        }
        else
        {
            uint cd = depth % 2;
            if(node->point[cd] > point[cd])
            {
                insertHelper(node->left, point, depth+1, id);
            }
            else
            {
                insertHelper(node->right, point, depth+1, id);
            }
        }

    }

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        insertHelper(root, point, 0, id);
	}

    void searchHelper(Node* &node, std::vector<int>* ids, std::vector<float> target, uint depth, float distTol)
    {
        if(node!=NULL)
        {
            float dx = node->point[0] - target[0];
            float dy = node->point[1] - target[1];
            if((node->point[0] >= (target[0] - distTol))  && (node->point[0] <= (target[0] + distTol)) && (node->point[1] >= (target[1] - distTol)) && (node->point[1] <= (target[1] + distTol)) )
            {
                float dist = sqrt(dx*dx + dy*dy);
                if (dist <= distTol)
                {
                    ids->push_back(node->id);
                }
            }
            uint cd = depth % 2;
            if(node->point[cd] > target[cd] - distTol)
            {
                searchHelper(node->left, ids, target, depth+1, distTol);
            }
            if(node->point[cd] < target[cd] + distTol)
            {
                searchHelper(node->right, ids, target, depth+1, distTol);
            }

        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;                
        searchHelper(root, &ids, target, 0, distanceTol);
		return ids;
	}
	

};




