#ifndef __A_LOCAL_PLANNER_H__
#define __A_LOCAL_PLANNER_H__

#include <dynamic_reconfigure/server.h>
#include <nav_core/base_local_planner.h>

#include <memory>
#include <string>

#include "move_pguard/LocalPlannerConfig.h"

namespace move_pguard
{
namespace local
{
/**
 * Abstract class for local planner using `dynamic_reconfigure`
 */
class ALocalPlanner : public nav_core::BaseLocalPlanner
{
public:
  ALocalPlanner(); /**< Construct a new ALocalPlanner */

  virtual ~ALocalPlanner(); /**< Virtual destructor for inheritance */

  /**
   * Initialize the `dynamic_reconfigure` server
   *
   * @param name The name to give to the server
   */
  void initializeConfigServer(std::string name);

protected:
  /**
   * Callback used by the `dynamic_reconfigure` server when a new configuration
   * is received
   *
   * @param config The new configuration
   * @param level The result of ORing together all of level values of the
   *              parameters that have changed
   */
  virtual void configCallback(LocalPlannerConfig& config, unsigned int level) = 0;

private:
  /** The `dynamic_reconfigure` server */
  std::shared_ptr<dynamic_reconfigure::Server<LocalPlannerConfig>> cfg_server_;
};
}  // namespace local
}  // namespace move_pguard
#endif  // __A_LOCAL_PLANNER_H__
