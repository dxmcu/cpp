#pragma once

class MatchCost {
public:
  MatchCost(size_t ridx, size_t cidx, double cost);

  // @brief access RowIdx
  size_t RowIdx() const;
  // @brief access ColIdx
  size_t ColIdx() const;

  // @brief access Cost
  double Cost() const;

  friend bool operator<(const MatchCost& m1, const MatchCost& m2);
  friend std::ostream& operator<<(std::ostream& os, const MatchCost& m);

private:
  size_t row_idx_ = 0;
  size_t col_idx_ = 0;
  double cost_ = 0.0;
};
