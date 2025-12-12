#pragma once

namespace atlas::livox {

/**
 * @brief Livox adapter entry point.
 *
 * This module provides Livox-specific implementations
 * without polluting the ATLAS core with vendor SDK headers.
 */
void probe();

}  // namespace atlas::livox