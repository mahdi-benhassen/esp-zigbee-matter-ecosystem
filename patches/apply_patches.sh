#!/bin/bash
# =============================================================================
# apply_patches.sh
# Patches managed_components after they are downloaded by the IDF component
# manager. These patches fix C++23 / GCC 14.2 compatibility issues in the
# esp_matter component that have not yet been fixed upstream.
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# ---------------------------------------------------------------------------
# Patch 1: closure-control-cluster-objects.h
#   GCC 14.2 with -std=gnu++2b cannot resolve std::optional<T>::operator==
#   when T has operator== defined only as a member function. We add a
#   free-standing (non-member) operator== so ADL finds it for std::optional.
# ---------------------------------------------------------------------------
patch_closure_control() {
    local GATEWAY_MC="$PROJECT_ROOT/gateway/main_soc/managed_components"
    local TARGET_FILE="$GATEWAY_MC/espressif__esp_matter/connectedhomeip/connectedhomeip/src/app/clusters/closure-control-server/closure-control-cluster-objects.h"

    if [ ! -f "$TARGET_FILE" ]; then
        echo "[PATCH] Skipping closure-control patch: file not found"
        return 0
    fi

    # Check if patch is already applied
    if grep -q "Free-standing operator== for std::optional compatibility" "$TARGET_FILE"; then
        echo "[PATCH] closure-control-cluster-objects.h: already patched"
        return 0
    fi

    echo "[PATCH] Patching closure-control-cluster-objects.h for C++23 compatibility..."

    # Insert free-standing operator== right before the closing namespace braces
    sed -i '/^} \/\/ namespace ClosureControl$/i\
\
/* ---------- Free-standing operator== for std::optional compatibility ---------- */\
/* GCC 14.2 with C++23 cannot find member operator== through std::optional.       */\
/* These free functions are found via ADL and satisfy the requirement.             */\
inline bool operator==(const GenericOverallCurrentState \& lhs,\
                       const GenericOverallCurrentState \& rhs)\
{\
    return lhs.position == rhs.position \&\& lhs.latch == rhs.latch \&\&\
           lhs.speed == rhs.speed \&\& lhs.secureState == rhs.secureState;\
}\
\
inline bool operator==(const GenericOverallTargetState \& lhs,\
                       const GenericOverallTargetState \& rhs)\
{\
    return lhs.position == rhs.position \&\& lhs.latch == rhs.latch \&\&\
           lhs.speed == rhs.speed;\
}' "$TARGET_FILE"

    echo "[PATCH] closure-control-cluster-objects.h: patched successfully"
}

# ---------------------------------------------------------------------------
# Apply all patches
# ---------------------------------------------------------------------------
echo "=== Applying managed_components patches ==="
patch_closure_control
echo "=== All patches applied ==="
