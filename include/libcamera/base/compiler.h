/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Compiler support
 */

#pragma once

#ifdef __nodiscard
#undef __nodiscard
#endif

#if __cplusplus >= 201703L
#define __nodiscard		[[nodiscard]]
#else
#define __nodiscard
#endif
