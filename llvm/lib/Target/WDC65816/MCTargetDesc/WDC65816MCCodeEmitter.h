//===-- WDC65816MCCodeEmitter.h - WDC65816 Code Emitter -----------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains the declarations for the code emitter which are useful
/// outside of the emitter itself.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_WDC65816_MCTARGETDESC_WDC65816MCCODEEMITTER_H
#define LLVM_LIB_TARGET_WDC65816_MCTARGETDESC_WDC65816MCCODEEMITTER_H

#include <cstdint>

namespace llvm {
namespace WDC65816 {

const uint8_t *getMCInstrBeads(unsigned);

} // namespace WDC65816
} // namespace llvm

#endif // LLVM_LIB_TARGET_WDC65816_MCTARGETDESC_WDC65816MCCODEEMITTER_H
