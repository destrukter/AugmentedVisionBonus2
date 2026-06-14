#pragma once

#include <cstdint>

namespace avb {

/// Opaque identifier used for every stored asset and assignment.
/// 0 is reserved to mean "no/invalid id".
using Id = std::uint64_t;

inline constexpr Id kInvalidId = 0;

} // namespace avb
