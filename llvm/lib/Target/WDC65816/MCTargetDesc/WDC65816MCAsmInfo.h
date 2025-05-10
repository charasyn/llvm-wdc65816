//===-- WDC65816MCAsmInfo.h - WDC65816 Asm Info -------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains the declarations of the WDC65816 MCAsmInfo properties.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_WDC65816_MCTARGETDESC_WDC65816MCASMINFO_H
#define LLVM_LIB_TARGET_WDC65816_MCTARGETDESC_WDC65816MCASMINFO_H

#include "llvm/MC/MCAsmInfoELF.h"

namespace llvm {
class Triple;

class WDC65816ELFMCAsmInfo : public MCAsmInfoELF {
  void anchor() override;

public:
  explicit WDC65816ELFMCAsmInfo(const Triple &Triple);
};

} // namespace llvm

#endif // LLVM_LIB_TARGET_WDC65816_MCTARGETDESC_WDC65816MCASMINFO_H
