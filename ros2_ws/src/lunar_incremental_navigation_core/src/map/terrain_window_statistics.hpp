#pragma once
#include <cstdint>
#include <deque>
#include <optional>
#include <set>
namespace lunar::incremental_navigation::map {
// 窗口统计只保留实际参与稳健地面梯度的中位数与完整性计数。
// 未知样本不填零，但仍占据窗口的位置。
class SlidingTerrainMedian {
 std::deque<std::optional<double>> order_;
 std::multiset<double> low_,high_;
 // FIFO移出的节点供下一次入窗复用；再平衡直接转移节点，避免逐格分配/释放。
 std::multiset<double>::node_type spare_;
 void Insert(std::multiset<double>& side,double value){
  if(spare_){spare_.value()=value;side.insert(std::move(spare_));}
  else side.insert(value);
 }
 void Balance(){
  auto target=(low_.size()+high_.size()+1)/2;
  while(low_.size()>target){auto it=std::prev(low_.end());high_.insert(low_.extract(it));}
  while(low_.size()<target){auto it=high_.begin();low_.insert(high_.extract(it));}
 }
 public:
 void Push(std::optional<double> value){order_.push_back(value);if(value){if(low_.empty()||*value<=*low_.rbegin())Insert(low_,*value);else Insert(high_,*value);Balance();}}
 void Pop(){if(order_.empty())return;auto v=order_.front();order_.pop_front();if(v){auto it=low_.find(*v);if(it!=low_.end())spare_=low_.extract(it);else {auto high=high_.find(*v);if(high!=high_.end())spare_=high_.extract(high);}Balance();}}
 std::optional<double> Median()const{if(low_.empty())return {};if(low_.size()==high_.size())return .5*(*low_.rbegin())+.5*(*high_.begin());return *low_.rbegin();}
};
class SlidingTerrainCount {
 std::deque<std::uint64_t> order_;std::uint64_t sum_{};
 public:void Push(std::uint64_t n){order_.push_back(n);sum_+=n;}
 void Pop(){if(!order_.empty()){sum_-=order_.front();order_.pop_front();}}
 std::uint64_t Total()const{return sum_;}
};
}
