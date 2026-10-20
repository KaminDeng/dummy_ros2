/**
 * @file limited_goal_budget_tests.cpp
 * @brief 核绑定boot/session的一次性1或2目标预算。
 *
 * 纯值对象测试；成功consume计的是准入次数，不代表目标执行成功。fail永久锁住，configure不可补充预算；assert须启用。
 */
#include <cassert>
#include "dummy_motion_client/limited_goal_budget.hpp"
using dummy_motion_client::LimitedGoalBudget;
/**
 * @brief 拒绝未配置/非法上限/零身份/重配置/错身份，耗尽后不再放行。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 独立对象分别核单次/双次消费以及已用一次后的故障锁，consumed不会因fail清零。
 */
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
