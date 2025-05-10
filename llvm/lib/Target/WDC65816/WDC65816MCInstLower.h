//===-- WDC65816MCInstLower.h - Lower MachineInstr to MCInst --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains code to lower WDC65816 MachineInstrs to their
/// corresponding MCInst records.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_WDC65816_WDC65816MCINSTLOWER_H
#define LLVM_LIB_TARGET_WDC65816_WDC65816MCINSTLOWER_H

#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
class MCContext;
class MCInst;
class MCOperand;
class MachineInstr;
class MachineFunction;
class WDC65816AsmPrinter;

/// This class is used to lower an MachineInstr into an MCInst.
class WDC65816MCInstLower {
  typedef MachineOperand::MachineOperandType MachineOperandType;
  MCContext &Ctx;
  MachineFunction &MF;
  const TargetMachine &TM;
  const MCAsmInfo &MAI;
  WDC65816AsmPrinter &AsmPrinter;

public:
  WDC65816MCInstLower(MachineFunction &MF, WDC65816AsmPrinter &AP);

  /// Lower an MO_GlobalAddress or MO_ExternalSymbol operand to an MCSymbol.
  MCSymbol *GetSymbolFromOperand(const MachineOperand &MO) const;

  MCOperand LowerSymbolOperand(const MachineOperand &MO, MCSymbol *Sym) const;

  std::optional<MCOperand> LowerOperand(const MachineInstr *MI,
                                        const MachineOperand &MO) const;

  void Lower(const MachineInstr *MI, MCInst &OutMI) const;
};
} // namespace llvm

#endif // LLVM_LIB_TARGET_WDC65816_WDC65816MCINSTLOWER_H
