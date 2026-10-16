#include <cassert>
#include "dummy_motion_client/limited_goal_budget.hpp"
using dummy_motion_client::LimitedGoalBudget;
int main() {
 LimitedGoalBudget b; assert(!b.consume(1,2)); assert(!b.configure(0,1,2));
 assert(!b.configure(3,1,2)); assert(!b.configure(1,0,2)); assert(b.configure(2,1,2));
 assert(!b.configure(2,1,2)); assert(!b.consume(2,2)); assert(!b.consume(1,3));
 assert(b.available()); assert(b.consume(1,2)); assert(b.consume(1,2));
 assert(!b.available()); assert(!b.consume(1,2)); assert(b.consumed()==2);
 LimitedGoalBudget one; assert(one.configure(1,1,2)); assert(one.consume(1,2));
 one.fail(); assert(!one.available()); assert(!one.consume(1,2)); assert(one.consumed()==1);
 LimitedGoalBudget failed; assert(failed.configure(2,1,2)); assert(failed.consume(1,2));
 failed.fail(); assert(!failed.consume(1,2)); assert(failed.consumed()==1);
}
