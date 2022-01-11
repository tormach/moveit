//
// Created by alexander on 1/20/22.
//

#pragma once

#include <string>
#include <mutex>

namespace pilz_industrial_motion_planner
{

class ErrorDetailsContainer
{
public:
  ErrorDetailsContainer();

  const std::string& getErrorMessage() const;
  void setErrorMessage(const std::string& errorMessage);

  const int32_t& getErrorCode() const;
  void setErrorCode(const int32_t& errorCode);

private:
  std::string error_message_;
  int32_t error_code_;
  mutable std::mutex mutex_;
};

}
