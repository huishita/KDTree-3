/**
* File: KDTree.h
* Author: Hyounggap An
* ------------------------
* An interface representing a kd-tree in some number of dimensions. The tree
* can be constructed from a set of data and then queried for membership and
* nearest neighbors.
*/

#ifndef KDTREE_INCLUDED
#define KDTREE_INCLUDED
#include <GL/freeglut.h>
#include "Point.h"
#include "BoundedPQueue.h"
#include <stdexcept>
#include <cmath>
#include <limits>

// "using namespace" in a header file is conventionally frowned upon, but I'm
// including it here so that you may use things like size_t without having to
// type std::size_t every time.
using namespace std;

template <size_t N, typename ElemType>
class KDTree {
public:
	// Constructor: KDTree();
	// Usage: KDTree<3, int> myTree;
	// ----------------------------------------------------
	// Constructs an empty KDTree.
	KDTree();

	// Destructor: ~KDTree()
	// Usage: (implicit)
	// ----------------------------------------------------
	// Cleans up all resources used by the KDTree.
	~KDTree();

	// KDTree(const KDTree& rhs);
	// KDTree& operator=(const KDTree& rhs);
	// Usage: KDTree<3, int> one = two;
	// Usage: one = two;
	// -----------------------------------------------------
	// Deep-copies the contents of another KDTree into this one.
	KDTree(const KDTree& rhs);
	KDTree(vector < pair<Point<N>, ElemType> >& pointCloud);
	KDTree& operator=(const KDTree& rhs);

	// size_t dimension() const;
	// Usage: size_t dim = kd.dimension();
	// ----------------------------------------------------
	// Returns the dimension of the points stored in this KDTree.
	size_t dimension() const;

	// size_t size() const;
	// bool empty() const;
	// Usage: if (kd.empty())
	// ----------------------------------------------------
	// Returns the number of elements in the kd-tree and whether the tree is
	// empty.
	size_t size() const;
	bool empty() const;

	// bool contains(const Point<N>& pt) const;
	// Usage: if (kd.contains(pt))
	// ----------------------------------------------------
	// Returns whether the specified point is contained in the KDTree.
	bool contains(const Point<N>& pt) const;

	// void insert(const Point<N>& pt, const ElemType& value);
	// Usage: kd.insert(v, "This value is associated with v.");
	// ----------------------------------------------------
	// Inserts the point pt into the KDTree, associating it with the specified
	// value. If the element already existed in the tree, the new value will
	// overwrite the existing one.
	void insert(const Point<N>& pt, const ElemType& value);

	// ElemType& operator[](const Point<N>& pt);
	// Usage: kd[v] = "Some Value";
	// ----------------------------------------------------
	// Returns a reference to the value associated with point pt in the KDTree.
	// If the point does not exist, then it is added to the KDTree using the
	// default value of ElemType as its key.
	ElemType& operator[](const Point<N>& pt);

	// ElemType& at(const Point<N>& pt);
	// const ElemType& at(const Point<N>& pt) const;
	// Usage: cout << kd.at(v) << endl;
	// ----------------------------------------------------
	// Returns a reference to the key associated with the point pt. If the point
	// is not in the tree, this function throws an out_of_range exception.
	ElemType& at(const Point<N>& pt);
	const ElemType& at(const Point<N>& pt) const;

	// ElemType kNNValue(const Point<N>& key, size_t k) const
	// Usage: cout << kd.kNNValue(v, 3) << endl;
	// ----------------------------------------------------
	// Given a point v and an integer k, finds the k points in the KDTree
	// nearest to v and returns the most common value associated with those
	// points. In the event of a tie, one of the most frequent value will be
	// chosen.
	ElemType kNNValue(const Point<N>& key, size_t k) const;

	// Traverse the kd-tree in-order
	void inOrderTraverse(bool draw = false) const;

	// Draw operations for openGL to visualize the tree
	void VisualizeBBox(size_t lv, size_t maxLv) const;

	const Point<N>* min(size_t dim) const;
	const Point<N>* max(size_t dim) const;

private:
	// TODO: Add implementation details here.
	size_t count;
	struct Node{
		Node(){ left = right = nullptr; }
		~Node(){ delete left; delete right; }
		Node(const Point<N>& _key, const ElemType& _value) :key(_key), value(_value)
		{
			left = right = nullptr;
		}
		Point<N> key;
		Point<N> Bmin, Bmax;
		ElemType value;
		Node *left, *right;
	};
	Node* root;
	// sub routine search the exact node
	// return nullptr if pt does not exist
	Node* findNode(const Point<N>& pt) const;

	// sub routine for deep copy
	Node* deepcopy(const Node* const src);

	// sub routine used in operator[]
	// generate node and insert in the tree
	void insert(Node& node);

	// inorder traversal sub routine
	void inorder(const Node* child, bool draw = false) const;

	// nearest neighbor search
	void nnSearch(const Node* const curNode, const Point<N>& key, BoundedPQueue<ElemType>& pQueue, size_t spldim) const;

	// custom comparator for sorting vector of pairs
	struct sort_pred{
		bool operator()(const pair<Point<N>, ElemType>& left, const pair<Point<N>, ElemType>& right)
		{
			return left.first[0] < right.first[0];
		}
	}comp;

	// for finding min and max
	const Point<N>* findMin(const Node* node, size_t dim, size_t cd) const;
	const Point<N>* findMax(const Node* node, size_t dim, size_t cd) const;

	// helper function
	void renderCube(const Point<N>& minp, const Point<N>& maxp) const;

	// void compute bounding box of the tree
	void computeBBox(Node* node, const Point<N>& bmin, const Point<N>& bmax, size_t dim);

	// visualize helper function
	void renderBBox(const Node* node, size_t lv, size_t maxLv) const;
};

/** KDTree class implementation details */

template <size_t N, typename ElemType>
KDTree<N, ElemType>::KDTree() : count(0) {
	// Create an empty KDTree
	root = nullptr;
}

template <size_t N, typename ElemType>
KDTree<N, ElemType>::KDTree(vector < pair<Point<N>, ElemType> >& pointCloud) : count(0)
{
	root = nullptr;
	sort(pointCloud.begin(), pointCloud.end(), comp);
	size_t len = pointCloud.size();
	size_t median = len / 2;
	insert(pointCloud[median].first, pointCloud[median].second);
	for (size_t i = 0; i < median; i++)
		insert(pointCloud[i].first, pointCloud[i].second);
	for (size_t i = median + 1; i < len; i++)
		insert(pointCloud[i].first, pointCloud[i].second);

	computeBBox(root, Point<N>(), Point<N>(), 0);
}

template <size_t N, typename ElemType>
KDTree<N, ElemType>::~KDTree() {
	// Release all resources KDTree kept
	delete root;
}

template <size_t N, typename ElemType>
typename KDTree<N, ElemType>::Node* KDTree<N, ElemType>::deepcopy(const Node* const src)
{
	Node* ret = new Node(src->key, src->value);
	if (src->left)
		ret->left = deepcopy(src->left);
	if (src->right)
		ret->right = deepcopy(src->right);
	return ret;
}

template <size_t N, typename ElemType>
KDTree<N, ElemType>::KDTree(const KDTree& rhs)
{
	if (rhs.empty())
	{
		KDTree();
	}
	else
	{
		count = rhs.size();
		root = deepcopy(rhs.root);
	}
}

template <size_t N, typename ElemType>
KDTree<N, ElemType>& KDTree<N, ElemType>::operator=(const KDTree& rhs)
{
	if (rhs.empty())
	{
		count = 0;
		root = nullptr;
		return *this;
	}
	else
	{
		count = rhs.size();
		root = deepcopy(rhs.root);
		return *this;
	}
}

template <size_t N, typename ElemType>
size_t KDTree<N, ElemType>::dimension() const {
	// return KDTree's dimension
	return N;
}

template <size_t N, typename ElemType>
size_t KDTree<N, ElemType>::size() const
{
	// return the number of nodes KDTree contains
	return count;
}
template <size_t N, typename ElemType>
bool KDTree<N, ElemType>::empty() const
{
	// return zero if KDTree is empty
	return (count == 0);
}

template <size_t N, typename ElemType>
bool KDTree<N, ElemType>::contains(const Point<N>& pt) const
{
	return findNode(pt) != nullptr;
}

template <size_t N, typename ElemType>
void KDTree<N, ElemType>::insert(const Point<N>& pt, const ElemType& value)
{
	// insert a node in KDTree
	if (!root)
	{
		root = new Node(pt, value);
		++count;
		return;
	}
	Node* curNode = root;
	size_t dim = 0;
	while (curNode)
	{
		if (curNode->key == pt)
		{
			curNode->value = value;
			return;
		}
		else
		{
			if (pt[dim] < curNode->key[dim])
			{
				// left
				if (curNode->left)
					curNode = curNode->left;
				else
				{
					Node* cNode = new Node(pt, value);
					curNode->left = cNode;
					++count;
					return;
				}
			}
			else
			{
				// right
				if (curNode->right)
					curNode = curNode->right;
				else
				{
					Node* cNode = new Node(pt, value);
					curNode->right = cNode;
					++count;
					return;
				}
			}
		}
		dim = (dim + 1) % N;
	}
}

template <size_t N, typename ElemType>
typename KDTree<N, ElemType>::Node* KDTree<N, ElemType>::findNode(const Point<N>& pt) const
{
	Node* curNode = root;
	size_t dim = 0;
	while (curNode)
	{
		if (curNode->key == pt)
			return curNode;
		if (pt[dim] < curNode->key[dim])
			curNode = curNode->left;
		else
			curNode = curNode->right;
		dim = (dim + 1) % N;
	}
	return nullptr;
}

template <size_t N, typename ElemType>
void KDTree<N, ElemType>::insert(Node& node)
{
	// insert a node in KDTree
	if (!root)
	{
		root = &node;
		++count;
		return;
	}
	Node* curNode = root;
	size_t dim = 0;
	while (curNode)
	{
		if (curNode->key == node.key)
		{
			curNode->value = node.value;
			return;
		}
		else
		{
			if (node.key[dim] < curNode->key[dim])
			{
				// left
				if (curNode->left)
					curNode = curNode->left;
				else
				{
					curNode->left = &node;
					++count;
					return;
				}
			}
			else
			{
				// right
				if (curNode->right)
					curNode = curNode->right;
				else
				{
					curNode->right = &node;
					++count;
					return;
				}
			}
		}
		dim = (dim + 1) % N;
	}
}

template <size_t N, typename ElemType>
ElemType& KDTree<N, ElemType>::operator[](const Point<N>& pt)
{
	Node* target = const_cast<Node*>(findNode(pt));
	if (!target)
	{
		target = new Node();
		target->key = pt;
		insert(*target);
	}
	return target->value;
}

template <size_t N, typename ElemType>
ElemType& KDTree<N, ElemType>::at(const Point<N>& pt)
{
	Node* target = findNode(pt);
	if (!target)
		throw out_of_range("The point does not exist in the tree");
	return target->value;
}

template <size_t N, typename ElemType>
const ElemType& KDTree<N, ElemType>::at(const Point<N>& pt) const
{
	Node* target = findNode(pt);
	if (!target)
		throw out_of_range("The point does not exist in the tree");
	return target->value;
}

template <size_t N, typename ElemType>
ElemType KDTree<N, ElemType>::kNNValue(const Point<N>& key, size_t k) const
{
	BoundedPQueue<ElemType> knn(k);
	nnSearch(root, key, knn, 0);
	ElemType* elems = new ElemType[k];
	size_t* count = new size_t[k];
	for (int i = 0; i<k; i++)
		count[i] = 0;
	size_t idx = 0, maxCount = 0;
	ElemType tmp, mostFrequent;
	bool found;
	// find most frequently occurring value
	while (!knn.empty())
	{
		tmp = knn.dequeueMin();
		found = false;
		for (size_t k = 0; k<idx; k++)
		{
			if (elems[k] == tmp)
			{
				count[k]++;
				found = true;
				break;
			}
		}
		if (!found)
		{
			elems[idx] = tmp;
			count[idx++] = 1;
		}
	}
	for (size_t i = 0; i<idx; i++)
	{
		if (maxCount < count[i])
		{
			maxCount = count[i];
			mostFrequent = ElemType(elems[i]);
		}
	}
	delete[] count;
	delete[] elems;
	return mostFrequent;
}

template <size_t N, typename ElemType>
void KDTree<N, ElemType>::nnSearch(const Node* const curNode, const Point<N>& key, BoundedPQueue<ElemType>& pQueue, size_t spldim) const
{
	if (!curNode)
		return;
	pQueue.enqueue(curNode->value, Distance(curNode->key, key));
	if (key[spldim] < curNode->key[spldim])
	{
		nnSearch(curNode->left, key, pQueue, (spldim + 1) % N);
		if (pQueue.size() < pQueue.maxSize() ||
			fabs(curNode->key[spldim] - key[spldim]) < pQueue.worst())
			nnSearch(curNode->right, key, pQueue, (spldim + 1) % N);
	}
	else
	{
		nnSearch(curNode->right, key, pQueue, (spldim + 1) % N);
		if (pQueue.size() < pQueue.maxSize() ||
			fabs(curNode->key[spldim] - key[spldim]) < pQueue.worst())
			nnSearch(curNode->left, key, pQueue, (spldim + 1) % N);
	}
}

template<size_t N, typename ElemType >
void KDTree<N, ElemType>::inOrderTraverse(bool draw) const
{
	if (draw)
		glVertex3d(root->key[0], root->key[1], root->key[2]);
	inorder(root, draw);
}

template<size_t N, typename ElemType >
void KDTree<N, ElemType>::inorder(const Node* child, bool draw) const
{
	if (child->left)
		inorder(child->left, draw);
	// print node
	if (draw)
	{
		glVertex3d(child->key[0], child->key[1], child->key[2]);
	}
		

	if (child->right)
		inorder(child->right, draw);
}

template<size_t N, typename ElemType >
void KDTree<N, ElemType>::VisualizeBBox(size_t lv, size_t maxLv) const
{
	renderBBox(root, lv, maxLv);
}

template<size_t N, typename ElemType >
void KDTree<N, ElemType>::renderBBox(const Node* node, size_t lv, size_t maxLv) const
{
	if (lv == maxLv)
	{
		renderCube(node->Bmin, node->Bmax);
		return;
	}
	if (node->left)
		renderBBox(node->left, lv + 1, maxLv);
	if (node->right)
		renderBBox(node->right, lv + 1, maxLv);
	if (!node->left || !node->right)
		renderCube(node->Bmin, node->Bmax);
}

template<size_t N, typename ElemType >
void KDTree<N, ElemType>::computeBBox(Node* node, const Point<N>& bmin, const Point<N>& bmax, size_t dim)
{
	if (!node)
		return;

	if (node == root)
	{
		Point<N> Bmin, Bmax;
		Bmin[0] = (*min(0))[0];
		Bmin[1] = (*min(1))[1];
		Bmin[2] = (*min(2))[2];
		Bmax[0] = (*max(0))[0];
		Bmax[1] = (*max(1))[1];
		Bmax[2] = (*max(2))[2];
		node->Bmin = Bmin;
		node->Bmax = Bmax;
		if (node->left)
		{
			Point<N> leftMax(Bmax);
			leftMax[0] = node->key[0];
			computeBBox(node->left, Bmin, leftMax, (dim + 1) % N);
		}
		if (node->right)
		{
			Point<N> rightMin(Bmin);
			rightMin[0] = node->key[0];
			computeBBox(node->right, rightMin, Bmax, (dim + 1) % N);
		}
	}
	else
	{
		node->Bmin = bmin;
		node->Bmax = bmax;
		if (node->left)
		{
			Point<N> leftMax(bmax);
			leftMax[dim] = node->key[dim];
			computeBBox(node->left, bmin, leftMax, (dim + 1) % N);
		}
		if (node->right)
		{
			Point<N> rightMin(bmin);
			rightMin[dim] = node->key[dim];
			computeBBox(node->right, rightMin, bmax, (dim + 1) % N);
		}
	}
}

template<size_t N, typename ElemType>
void KDTree<N, ElemType>::renderCube(const Point<N>& minp, const Point<N>& maxp) const
{
	glBegin(GL_LINE_LOOP);
	// bottom
	glVertex3d(minp[0], minp[1], minp[2]);
	glVertex3d(maxp[0], minp[1], minp[2]);
	glVertex3d(maxp[0], minp[1], maxp[2]);
	glVertex3d(minp[0], minp[1], maxp[2]);
	glEnd();
	glBegin(GL_LINE_LOOP);
	// front
	glVertex3d(minp[0], minp[1], minp[2]);
	glVertex3d(maxp[0], minp[1], minp[2]);
	glVertex3d(maxp[0], maxp[1], minp[2]);
	glVertex3d(minp[0], maxp[1], minp[2]);
	glEnd();
	glBegin(GL_LINE_LOOP);
	// top
	glVertex3d(minp[0], maxp[1], minp[2]);
	glVertex3d(maxp[0], maxp[1], minp[2]);
	glVertex3d(maxp[0], maxp[1], maxp[2]);
	glVertex3d(minp[0], maxp[1], maxp[2]);
	glEnd();
	glBegin(GL_LINE_LOOP);
	// BACK
	glVertex3d(maxp[0], maxp[1], maxp[2]);
	glVertex3d(minp[0], maxp[1], maxp[2]);
	glVertex3d(minp[0], minp[1], maxp[2]);
	glVertex3d(maxp[0], minp[1], maxp[2]);
	glEnd();
	glBegin(GL_LINE_LOOP);
	// LEFT
	glVertex3d(minp[0], minp[1], minp[2]);
	glVertex3d(minp[0], maxp[1], minp[2]);
	glVertex3d(minp[0], maxp[1], maxp[2]);
	glVertex3d(minp[0], minp[1], maxp[2]);
	glEnd();
	glBegin(GL_LINE_LOOP);
	// RIGHT
	glVertex3d(maxp[0], minp[1], minp[2]);
	glVertex3d(maxp[0], minp[1], maxp[2]);
	glVertex3d(maxp[0], maxp[1], maxp[2]);
	glVertex3d(maxp[0], maxp[1], minp[2]);
	glEnd();
}

template<size_t N, typename ElemType >
inline const Point<N>* KDTree<N, ElemType>::min(size_t dim) const
{
	return findMin(root, dim, 0);
}

template<size_t N, typename ElemType >
inline const Point<N>* KDTree<N, ElemType>::max(size_t dim) const
{
	return findMax(root, dim, 0);
}

template<size_t N, typename ElemType >
const Point<N>* KDTree<N, ElemType>::findMin(const Node* node, size_t dim, size_t cd) const
{
	if (!node)
		return nullptr;
	if (cd == dim)
	{
		if (node->left)
			return findMin(node->left, dim, (cd + 1) % N);
		else
			return &node->key;
	}
	else
	{
		const Point<N>* l = findMin(node->left, dim, (cd + 1) % N);
		const Point<N>* r = findMin(node->right, dim, (cd + 1) % N);
		double lvalue, rvalue;
		if (!l)
			lvalue = numeric_limits<float>::max();
		else
			lvalue = (*l)[dim];
		if (!r)
			rvalue = numeric_limits<float>::max();
		else
			rvalue = (*r)[dim];
		if (lvalue < rvalue && lvalue < node->key[dim])
		{
			return l;
		}
		else if ( rvalue < node->key[dim])
		{
			return r;
		}
		else
		{
			return &node->key;
		}
	}
}

template<size_t N, typename ElemType >
const Point<N>* KDTree<N, ElemType>::findMax(const Node* node, size_t dim, size_t cd) const
{
	if (!node)
		return nullptr;
	if (cd == dim)
	{
		if (node->right)
			return findMax(node->right, dim, (cd + 1) % N);
		else
			return &node->key;
	}
	else
	{
		const Point<N>* l = findMax(node->left, dim, (cd + 1) % N);
		const Point<N>* r = findMax(node->right, dim, (cd + 1) % N);
		double lvalue, rvalue;
		if (!l)
			lvalue = numeric_limits<double>::min();
		else
			lvalue = (*l)[dim];
		if (!r)
			rvalue = numeric_limits<double>::min();
		else
			rvalue = (*r)[dim];
		if (lvalue > rvalue && lvalue > node->key[dim])
		{
			return l;
		}
		else if (rvalue > node->key[dim])
		{
			return r;
		}
		else
		{
			return &node->key;
		}
	}
}

#endif // KDTREE_INCLUDED
