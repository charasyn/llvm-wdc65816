//===-- WDC65816.h - Top-level interface for WDC65816 representation ----*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains the entry points for global functions defined in the
/// WDC65816 target library, as used by the LLVM JIT.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_WDC65816_WDC65816_H
#define LLVM_LIB_TARGET_WDC65816_WDC65816_H

namespace llvm {

class FunctionPass;
class InstructionSelector;
class WDC65816RegisterBankInfo;
class WDC65816Subtarget;
class WDC65816TargetMachine;
class PassRegistry;

/// This pass converts a legalized DAG into a WDC65816-specific DAG, ready for
/// instruction scheduling.
FunctionPass *createWDC65816ISelDag(WDC65816TargetMachine &TM);

/// Return a Machine IR pass that expands WDC65816-specific pseudo
/// instructions into a sequence of actual instructions. This pass
/// must run after prologue/epilogue insertion and before lowering
/// the MachineInstr to MC.
FunctionPass *createWDC65816ExpandPseudoPass();

/// This pass initializes a global base register for PIC on WDC65816.
FunctionPass *createWDC65816GlobalBaseRegPass();

/// Finds sequential MOVEM instruction and collapse them into a single one. This
/// pass has to be run after all pseudo expansions and prologue/epilogue
/// emission so that all possible MOVEM are already in place.
FunctionPass *createWDC65816CollapseMOVEMPass();

InstructionSelector *
createWDC65816InstructionSelector(const WDC65816TargetMachine &, const WDC65816Subtarget &,
                              const WDC65816RegisterBankInfo &);

void initializeWDC65816DAGToDAGISelLegacyPass(PassRegistry &);
void initializeWDC65816ExpandPseudoPass(PassRegistry &);
void initializeWDC65816GlobalBaseRegPass(PassRegistry &);
void initializeWDC65816CollapseMOVEMPass(PassRegistry &);

} // namespace llvm

#endif // LLVM_LIB_TARGET_WDC65816_WDC65816_H
