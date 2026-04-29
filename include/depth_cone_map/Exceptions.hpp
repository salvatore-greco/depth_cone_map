#ifndef EXCEPTIONS_HPP
#define EXCEPTIONS_HPP

#include <exception>
#include <stdexcept>
#include <string>
class NoFeatureFoundException : public std::runtime_error{
    public:
    explicit NoFeatureFoundException(const std::string& what_arg) : std::runtime_error(what_arg){};
};

#endif // EXCEPTIONS_HPP
