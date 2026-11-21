#pragma once

// Error handling foundation: re-export exstl's Result/Error so engine and
// game code share one result type without including exstl paths directly.

#include <exstl/errors.hpp>
#include <exstl/error_macros.hpp>

namespace rush {

using Error = exstl::Error;

template <typename T>
using Result = exstl::Result<T>;

using exstl::check_result;

}  // namespace rush
