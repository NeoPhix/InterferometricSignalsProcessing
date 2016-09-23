#ifndef SYMBOLIC_TREE_H
#define SYMBOLIC_TREE_H

#include <random>

namespace FilterTuning
{
	enum class LeafType
	{
		plus,
		minus,
		mul,
		div,
		digree,
		sqrt,
		exp,
		log,
		value,
		variable
	};

	class SymbolicTree
	{
	public:
		SymbolicTree();
		~SymbolicTree();

		void mutate();
		void cross(SymbolicTree *tree);
		double parse(double x);
	private:
		LeafType leafType;
		SymbolicTree *right;
		SymbolicTree *left;

		std::default_random_engine gen;
	};
}

#endif

