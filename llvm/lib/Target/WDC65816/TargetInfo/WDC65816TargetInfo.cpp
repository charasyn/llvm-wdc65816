//===-- WDC65816TargetInfo.cpp - WDC65816 Target Implementation ---------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains WDC65816 target initializer.
///
//===----------------------------------------------------------------------===//
#include "llvm/MC/TargetRegistry.h"

using namespace llvm;

namespace llvm {
Target &getTheWDC65816Target() {
  static Target TheWDC65816Target;
  return TheWDC65816Target;
}
} // namespace llvm

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeWDC65816TargetInfo() {
  RegisterTarget<Triple::m68k, /*HasJIT=*/true> X(
      getTheWDC65816Target(), "m68k", "Motorola 68000 family", "WDC65816");
}
