//
// Created by alexander on 1/20/22.
//
#include "pilz_industrial_motion_planner/error_details_container.h"

pilz_industrial_motion_planner::ErrorDetailsContainer::ErrorDetailsContainer()
{
}
const std::string& pilz_industrial_motion_planner::ErrorDetailsContainer::getErrorMessage() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return error_message_;
}
void pilz_industrial_motion_planner::ErrorDetailsContainer::setErrorMessage(const std::string& errorMessage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  error_message_ = errorMessage;
}
const int32_t& pilz_industrial_motion_planner::ErrorDetailsContainer::getErrorCode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return error_code_;
}
void pilz_industrial_motion_planner::ErrorDetailsContainer::setErrorCode(const int32_t& errorCode)
{
  std::lock_guard<std::mutex> lock(mutex_);
  error_code_ = errorCode;
}

//bool pilz_industrial_motion_planner::ErrorDetailsContainer
