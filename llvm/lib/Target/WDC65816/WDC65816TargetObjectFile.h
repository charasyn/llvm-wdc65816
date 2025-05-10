//===-- WDC65816ELFTargetObjectFile.h - WDC65816 Object Info ------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains declarations for WDC65816 ELF object file lowering.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_WDC65816_WDC65816TARGETOBJECTFILE_H
#define LLVM_LIB_TARGET_WDC65816_WDC65816TARGETOBJECTFILE_H

#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"

namespace llvm {
class WDC65816TargetMachine;
class WDC65816ELFTargetObjectFile : public TargetLoweringObjectFileELF {
  const WDC65816TargetMachine *TM;
  MCSection *SmallDataSection;
  MCSection *SmallBSSSection;

public:
  void Initialize(MCContext &Ctx, const TargetMachine &TM) override;
};
} // end namespace llvm

#endif // LLVM_LIB_TARGET_WDC65816_WDC65816TARGETOBJECTFILE_H
