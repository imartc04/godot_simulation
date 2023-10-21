#pragma once

#include <stdexception>


/**
 * Custom exception class for grpc clients
 * Inherits from std::runtime_error
 */
class CGRPCClientException : public std::runtime_error
{
  enum class::EGRPCClientExceptionType
  {
    UNINITIALIZED,
    CONFIGURATION_ERROR,
    CONNECTION_ERROR,
    UNKNOWN_ERROR
  } m_exception_type;



};
