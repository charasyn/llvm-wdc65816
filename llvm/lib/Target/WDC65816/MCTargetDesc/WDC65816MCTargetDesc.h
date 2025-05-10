//===-- WDC65816MCTargetDesc.h - WDC65816 Target Descriptions -----------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file provides WDC65816 specific target descriptions.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_WDC65816_MCTARGETDESC_WDC65816MCTARGETDESC_H
#define LLVM_LIB_TARGET_WDC65816_MCTARGETDESC_WDC65816MCTARGETDESC_H

#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/Support/DataTypes.h"

namespace llvm {
class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCRegisterInfo;
class MCSubtargetInfo;
class MCRelocationInfo;
class MCTargetOptions;
class Target;
class Triple;
class StringRef;
class raw_ostream;
class raw_pwrite_stream;

MCAsmBackend *createWDC65816AsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                   const MCRegisterInfo &MRI,
                                   const MCTargetOptions &Options);

MCCodeEmitter *createWDC65816MCCodeEmitter(const MCInstrInfo &MCII,
                                       MCContext &Ctx);

/// Construct an WDC65816 ELF object writer.
std::unique_ptr<MCObjectTargetWriter> createWDC65816ELFObjectWriter(uint8_t OSABI);

} // namespace llvm

// Defines symbolic names for WDC65816 registers. This defines a mapping from
// register name to register number.
#define GET_REGINFO_ENUM
#include "WDC65816GenRegisterInfo.inc"

// Defines symbolic names for the WDC65816 instructions.
#define GET_INSTRINFO_ENUM
#define GET_INSTRINFO_MC_HELPER_DECLS
#include "WDC65816GenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "WDC65816GenSubtargetInfo.inc"

#endif // LLVM_LIB_TARGET_WDC65816_MCTARGETDESC_WDC65816MCTARGETDESC_H
