#pragma once

#include <cassert>

#ifdef __cpp_exceptions
    #define PISAR_EXCEPTIONS_ENABLED
#else
    #define PISAR_EXCEPTIONS_DISABLED
#endif

#define PISAR_ASSERT(assertion, message) assert((assertion) && (message))

#ifdef PISAR_EXCEPTIONS_ENABLED
    #define PISAR_ASSERT_EXCEPTION(assertion, exception, message) \
    {                                                       \
        if ((assertion))                                    \
        {                                                   \
            throw (exception);                              \
        }                                                   \
    }
#else
    #define PISAR_ASSERT_EXCEPTION(assertion, exception, message) PISAR_ASSERT(assertion, message)
#endif

#define PISAR_ASSERT_BASIC_EXCEPTION(assertion, exception_type, message) \
    PISAR_ASSERT_EXCEPTION(assertion, exception_type(message), message)
