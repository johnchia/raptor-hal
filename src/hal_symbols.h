/*
 * hal_symbols.h -- dlsym helper shared by the module loaders
 *
 * divinus's src/hal/symbols.h, reduced to the one helper the loaders ever
 * took from it. It lives here rather than in sigmastar-headers because it
 * reports through raptor's logger: the headers repo describes the MI ABI and
 * nothing more, so anything naming HAL_LOG_* or RSS_ERR_* belongs on this
 * side of the boundary.
 *
 * One level above the backends because it is specific to none of them: nothing
 * in it knows a vendor or an SDK generation, and every backend loads symbols
 * the same way.
 *
 * Copyright (c) 2024 OpenIPC
 * SPDX-License-Identifier: MIT
 */

#ifndef HAL_SYMBOLS_H
#define HAL_SYMBOLS_H

#include "hal_internal.h"

#include <dlfcn.h>

/*
 * hal_symbol_load -- dlsym with a diagnostic.
 *
 * Returns NULL on failure, so call sites read as
 *   if (!(lib->fnFoo = (cast)hal_symbol_load(mod, lib->handle, "MI_Foo")))
 *       return RSS_ERR_NOTSUP;
 * Naming the module in the message matters: with a backend's worth of
 * libraries loaded, the symbol alone does not say which came up short.
 */
static inline void *hal_symbol_load(const char *module, void *handle, const char *symbol)
{
    void *function = dlsym(handle, symbol);

    if (!function) {
        HAL_LOG_ERR("%s: failed to acquire symbol %s", module, symbol);
        return NULL;
    }

    return function;
}

#endif /* HAL_SYMBOLS_H */
