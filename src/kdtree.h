
// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	// function to create a new node
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

	// double pointer method
	// pass in pointer to the node starting at root
	// when node is found that is null and can be replaced, then dereference the double pointer and assign to new created node
	// pass in memory address of root node, which was originally a pointer
	void insertHelperDoublePointer(Node** node, uint depth, std::vector<float> point, int id)
	{
		// when tree is empty
		// derefernce node to see its value
		// if this is actually the root and empty then set the point and id
		// this will also work if the current explored node is NULL
		if(*node==NULL)
			// redirect pointer to new data/ reassign passed in node to new data
			*node = new Node(point, id);
		else
		{
			// calculate the depth in the tree
			uint current_depth = depth % 2;

			// check if the point is less than the current focal node
			if(point[current_depth] < ((*node)->point[current_depth]))
				// if less, then push node to left of tree
				insertHelperDoublePointer(&((*node)->left), depth+1, point, id);
			else
				// must be larger so push to right of tree
				insertHelperDoublePointer(&((*node)->right), depth+1, point, id);
		}

	}

	void insertHelperPointerReference(Node*& node, uint depth, std::vector<float> point, int id)
	{
		// when tree is empty
		// reference of node is passed to function
		// node to see its value
		// if this is actually the root and empty then set the point and id
		// this will also work if the current explored node is NULL
		if(node==NULL)
			// redirect pointer to new data/ reassign passed in node to new data
			// assign values to node referece
			node = new Node(point, id);
		else
		{
			// calculate the depth in the tree - only positive depths so uint
			// %2 can determine if depth is odd or even
			uint current_depth = depth % 2;

			// check if the point is less than the current focal node - if its even
			// as even then x value is considered
			// point[current_depth] vector is either 0 or 1... 0 = even, 1 == odd
			// compare if it is less than passed in x[0] or y[1] value
			if(point[current_depth] < (node->point[current_depth]))
				// if less, then push node to left of tree, increment by one and store the point and id data
				insertHelperPointerReference((node->left), depth+1, point, id);
			else
				// must be larger so push to right of tree, increment by one and store the point and id data
				insertHelperPointerReference((node->right), depth+1, point, id);
		}

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		// call an insert helper to insert node - recurssive function
		// pass in memory address for root
		// depth is 0 as at root

		//insertHelperDoublePointer(&root, 0, point, id);
		insertHelperPointerReference(root, 0, point, id);

	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if(node!=NULL)
		{
			/*std::cout << "Node to compare: id: " << node->id << " XY: " << node->point[0] << ", " << node->point[1] << " with: " << target[0] << ", " << target[1] << std::endl;

			std::cout << 	"test greater than x: " << (node->point[0] >=(target[0] - distanceTol)) <<
							"\ntest less than x: "	<< (node->point[0] <=(target[0] + distanceTol)) <<
							"\ntest greater than y: " << (node->point[1] >=(target[1] - distanceTol)) <<
							"\ntest less than y: "	<< (node->point[1] <=(target[1] + distanceTol)) <<
							std::endl;*/

			// check node distance is within the bounds of the target tolerance
			if (	(node->point[0] >= (target[0] - distanceTol)) && 	// if node in in -x range
					(node->point[0] <= (target[0] + distanceTol)) && 	// if node in in +x range
					(node->point[1] >= (target[1] - distanceTol)) && 	// if node in in -y range
					(node->point[1] <= (target[1] + distanceTol))		// if node in in +y range
					)
			{


				// get the point distance in each axes
				float x = fabs(node->point[0] - target[0]);
				float y = fabs(node->point[1] - target[1]);

				//std::cout << "XY: " << x <<", " << y << std::endl;

				// as node is within range then get distance to node
				float distance = sqrt(x*x+y*y);
				//std::cout << "Distance between nodes: " << distance << std::endl;


				// check if the vector distance is within tolerance range
				if(distance <= distanceTol)
					// in range so add to list of nearests
					ids.push_back(node->id);

			}

			// check in node tree to flow left or right - recursive function to explore tree
			if((target[depth%2] - distanceTol) < node->point[depth%2])
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			if((target[depth%2] + distanceTol) > node->point[depth%2])
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		// function to find nearest nodes and update ids (by refernce) to hold list of nearest node ids
		searchHelper(target, root, 0, distanceTol, ids);



		return ids;
	}


};




