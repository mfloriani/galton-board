#pragma once

#include "Geometry2D.h"

#include <vector>

struct QuadTreeData 
{
	void*       object{ nullptr };
	Rectangle2D bounds;
	bool        flag;

	QuadTreeData() :
		flag(false) { }

	QuadTreeData(void* o, const Rectangle2D& b) :
		object(o), bounds(b), flag(false) { }
};

class QuadTreeNode
{
protected:
	std::vector<QuadTreeNode>  children;
	std::vector<QuadTreeData*> contents;
	int         currentDepth;
	static int  maxDepth;
	static int  maxObjectsPerNode;
	Rectangle2D nodeBounds;

public:
	QuadTreeNode(const Rectangle2D& bounds) :
		nodeBounds(bounds), currentDepth(0) { }

	inline bool IsLeaf() { return children.size() == 0; };
	int  NumObjects();
	void Insert(QuadTreeData& data);
	void Remove(QuadTreeData& data);
	void Update(QuadTreeData& data);
	void Shake();
	void Split();
	void Reset();
	std::vector<QuadTreeData*>Query(const Rectangle2D& area);

};

typedef QuadTreeNode QuadTree;
