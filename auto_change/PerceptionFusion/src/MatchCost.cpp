#include <ostream>
#include "MatchCost.h"


MatchCost::MatchCost(size_t ridx, size_t cidx, double cost)
	: row_idx_(ridx), col_idx_(cidx), cost_(cost) {}

size_t MatchCost::RowIdx() const { return row_idx_; }

size_t MatchCost::ColIdx() const { return col_idx_; }

double MatchCost::Cost() const { return cost_; }

bool operator<(const MatchCost& m1, const MatchCost& m2) {
	return m1.cost_ < m2.cost_;
}

std::ostream& operator<<(std::ostream& os, const MatchCost& m) {
	os << "MatchCost ridx:" << m.RowIdx() << " cidx:" << m.ColIdx()
		<< " Cost:" << m.Cost();
	return os;
}
