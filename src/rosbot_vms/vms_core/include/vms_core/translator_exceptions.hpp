#ifndef VMS_CORE__TRANSLATOR_EXCEPTIONS_HPP_
#define VMS_CORE__TRANSLATOR_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <memory>

namespace vms_core
{

class TranslatorException : public std::runtime_error
{
public:
  explicit TranslatorException(const std::string & description)
  : std::runtime_error(description) {}
};

class InvalidTranslator : public TranslatorException
{
public:
  explicit InvalidTranslator(const std::string & description)
  : TranslatorException(description) {}
};

class NoValidPathCouldBeFound : public TranslatorException
{
public:
  explicit NoValidPathCouldBeFound(const std::string & description)
  : TranslatorException(description) {}
};

class TranslatorTimedOut : public TranslatorException
{
public:
  explicit TranslatorTimedOut(const std::string & description)
  : TranslatorException(description) {}
};

class TranslatorCancelled : public TranslatorException
{
public:
  explicit TranslatorCancelled(const std::string & description)
  : TranslatorException(description) {}
};

}  // namespace vms_core

#endif  // VMS_CORE__TRANSLATOR_EXCEPTIONS_HPP_