//===-- WDC65816MCAsmInfo.cpp - WDC65816 Asm Properties -----------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains the definitions of the WDC65816 MCAsmInfo properties.
///
//===----------------------------------------------------------------------===//

#include "WDC65816MCAsmInfo.h"

#include "llvm/TargetParser/Triple.h"

using namespace llvm;

void WDC65816ELFMCAsmInfo::anchor() {}

WDC65816ELFMCAsmInfo::WDC65816ELFMCAsmInfo(const Triple &T) {
  CodePointerSize = 4;
  CalleeSaveStackSlotSize = 4;

  IsLittleEndian = false;

  // Debug Information
  SupportsDebugInformation = true;

  // Exceptions handling
  ExceptionsType = ExceptionHandling::DwarfCFI;

  UseMotorolaIntegers = true;
  CommentString = ";";
}
