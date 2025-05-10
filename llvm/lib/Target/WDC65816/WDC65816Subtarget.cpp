//===-- WDC65816Subtarget.cpp - WDC65816 Subtarget Information ----------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file implements the WDC65816 specific subclass of TargetSubtargetInfo.
///
//===----------------------------------------------------------------------===//

#include "WDC65816Subtarget.h"
#include "GISel/WDC65816CallLowering.h"
#include "GISel/WDC65816LegalizerInfo.h"
#include "GISel/WDC65816RegisterBankInfo.h"

#include "WDC65816.h"
#include "WDC65816MachineFunction.h"
#include "WDC65816RegisterInfo.h"
#include "WDC65816TargetMachine.h"

#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "m68k-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "WDC65816GenSubtargetInfo.inc"

extern bool FixGlobalBaseReg;

/// Select the WDC65816 CPU for the given triple and cpu name.
static StringRef selectWDC65816CPU(Triple TT, StringRef CPU) {
  if (CPU.empty() || CPU == "generic") {
    CPU = "M68000";
  }
  return CPU;
}

void WDC65816Subtarget::anchor() {}

WDC65816Subtarget::WDC65816Subtarget(const Triple &TT, StringRef CPU, StringRef FS,
                             const WDC65816TargetMachine &TM)
    : WDC65816GenSubtargetInfo(TT, CPU, /*TuneCPU*/ CPU, FS), TM(TM), TSInfo(),
      InstrInfo(initializeSubtargetDependencies(CPU, TT, FS, TM)),
      FrameLowering(*this, this->getStackAlignment()), TLInfo(TM, *this),
      TargetTriple(TT) {
  CallLoweringInfo.reset(new WDC65816CallLowering(*getTargetLowering()));
  Legalizer.reset(new WDC65816LegalizerInfo(*this));

  auto *RBI = new WDC65816RegisterBankInfo(*getRegisterInfo());
  RegBankInfo.reset(RBI);
  InstSelector.reset(createWDC65816InstructionSelector(TM, *this, *RBI));
}

const CallLowering *WDC65816Subtarget::getCallLowering() const {
  return CallLoweringInfo.get();
}

InstructionSelector *WDC65816Subtarget::getInstructionSelector() const {
  return InstSelector.get();
}

const LegalizerInfo *WDC65816Subtarget::getLegalizerInfo() const {
  return Legalizer.get();
}

const RegisterBankInfo *WDC65816Subtarget::getRegBankInfo() const {
  return RegBankInfo.get();
}

bool WDC65816Subtarget::isPositionIndependent() const {
  return TM.isPositionIndependent();
}

bool WDC65816Subtarget::isLegalToCallImmediateAddr() const { return true; }

WDC65816Subtarget &WDC65816Subtarget::initializeSubtargetDependencies(
    StringRef CPU, Triple TT, StringRef FS, const WDC65816TargetMachine &TM) {
  std::string CPUName = selectWDC65816CPU(TT, CPU).str();

  // Parse features string.
  ParseSubtargetFeatures(CPUName, CPUName, FS);

  // Initialize scheduling itinerary for the specified CPU.
  InstrItins = getInstrItineraryForCPU(CPUName);

  stackAlignment = 8;

  return *this;
}

//===----------------------------------------------------------------------===//
// Code Model
//
// Key assumptions:
//  - Whenever possible we use pc-rel encoding since it is smaller(16 bit) than
//    absolute(32 bit).
//  - GOT is reachable within 16 bit offset for both Small and Medium models.
//  - Code section is reachable within 16 bit offset for both models.
//
//  ---------------------+-------------------------+--------------------------
//                       |          Small          |          Medium
//                       +-------------------------+------------+-------------
//                       |   Static   |    PIC     |   Static   |    PIC
//  ---------------------+------------+------------+------------+-------------
//                branch |   pc-rel   |   pc-rel   |   pc-rel   |   pc-rel
//  ---------------------+------------+------------+------------+-------------
//           call global |    @PLT    |    @PLT    |    @PLT    |    @PLT
//  ---------------------+------------+------------+------------+-------------
//         call internal |   pc-rel   |   pc-rel   |   pc-rel   |   pc-rel
//  ---------------------+------------+------------+------------+-------------
//            data local |   pc-rel   |   pc-rel   |  ~pc-rel   |  ^pc-rel
//  ---------------------+------------+------------+------------+-------------
//       data local big* |   pc-rel   |   pc-rel   |  absolute  |  @GOTOFF
//  ---------------------+------------+------------+------------+-------------
//           data global |   pc-rel   |  @GOTPCREL |  ~pc-rel   |  @GOTPCREL
//  ---------------------+------------+------------+------------+-------------
//      data global big* |   pc-rel   |  @GOTPCREL |  absolute  |  @GOTPCREL
//  ---------------------+------------+------------+------------+-------------
//
// * Big data potentially cannot be reached within 16 bit offset and requires
//   special handling for old(x00 and x10) CPUs. Normally these symbols go into
//   separate .ldata section which mapped after normal .data and .text, but I
//   don't really know how this must be done for WDC65816 atm... will try to dig
//   this info out from GCC. For now CPUs prior to M68020 will use static ref
//   for Static Model and @GOT based references for PIC.
//
// ~ These are absolute for older CPUs for now.
// ^ These are @GOTOFF for older CPUs for now.
//===----------------------------------------------------------------------===//

/// Classify a blockaddress reference for the current subtarget according to how
/// we should reference it in a non-pcrel context.
unsigned char WDC65816Subtarget::classifyBlockAddressReference() const {
  // Unless we start to support Large Code Model branching is always pc-rel
  return WDC65816II::MO_PC_RELATIVE_ADDRESS;
}

unsigned char
WDC65816Subtarget::classifyLocalReference(const GlobalValue *GV) const {
  switch (TM.getCodeModel()) {
  default:
    llvm_unreachable("Unsupported code model");
  case CodeModel::Small:
  case CodeModel::Kernel: {
    return WDC65816II::MO_PC_RELATIVE_ADDRESS;
  }
  case CodeModel::Medium: {
    if (isPositionIndependent()) {
      // On M68020 and better we can fit big any data offset into dips field.
      if (atLeastM68020()) {
        return WDC65816II::MO_PC_RELATIVE_ADDRESS;
      }
      // Otherwise we could check the data size and make sure it will fit into
      // 16 bit offset. For now we will be conservative and go with @GOTOFF
      return WDC65816II::MO_GOTOFF;
    } else {
      if (atLeastM68020()) {
        return WDC65816II::MO_PC_RELATIVE_ADDRESS;
      }
      return WDC65816II::MO_ABSOLUTE_ADDRESS;
    }
  }
  }
}

unsigned char WDC65816Subtarget::classifyExternalReference(const Module &M) const {
  if (TM.shouldAssumeDSOLocal(nullptr))
    return classifyLocalReference(nullptr);

  if (isPositionIndependent())
    return WDC65816II::MO_GOTPCREL;

  return WDC65816II::MO_GOT;
}

unsigned char
WDC65816Subtarget::classifyGlobalReference(const GlobalValue *GV) const {
  return classifyGlobalReference(GV, *GV->getParent());
}

unsigned char WDC65816Subtarget::classifyGlobalReference(const GlobalValue *GV,
                                                     const Module &M) const {
  if (TM.shouldAssumeDSOLocal(GV))
    return classifyLocalReference(GV);

  switch (TM.getCodeModel()) {
  default:
    llvm_unreachable("Unsupported code model");
  case CodeModel::Small:
  case CodeModel::Kernel: {
    if (isPositionIndependent())
      return WDC65816II::MO_GOTPCREL;
    return WDC65816II::MO_PC_RELATIVE_ADDRESS;
  }
  case CodeModel::Medium: {
    if (isPositionIndependent())
      return WDC65816II::MO_GOTPCREL;

    if (atLeastM68020())
      return WDC65816II::MO_PC_RELATIVE_ADDRESS;

    return WDC65816II::MO_ABSOLUTE_ADDRESS;
  }
  }
}

unsigned WDC65816Subtarget::getJumpTableEncoding() const {
  if (isPositionIndependent()) {
    // The only time we want to use GOTOFF(used when with EK_Custom32) is when
    // the potential delta between the jump target and table base can be larger
    // than displacement field, which is True for older CPUs(16 bit disp)
    // in Medium model(can have large data way beyond 16 bit).
    if (TM.getCodeModel() == CodeModel::Medium && !atLeastM68020())
      return MachineJumpTableInfo::EK_Custom32;

    return MachineJumpTableInfo::EK_LabelDifference32;
  }

  // In non-pic modes, just use the address of a block.
  return MachineJumpTableInfo::EK_BlockAddress;
}

unsigned char
WDC65816Subtarget::classifyGlobalFunctionReference(const GlobalValue *GV) const {
  return classifyGlobalFunctionReference(GV, *GV->getParent());
}

unsigned char
WDC65816Subtarget::classifyGlobalFunctionReference(const GlobalValue *GV,
                                               const Module &M) const {
  // local always use pc-rel referencing
  if (TM.shouldAssumeDSOLocal(GV))
    return WDC65816II::MO_NO_FLAG;

  // If the function is marked as non-lazy, generate an indirect call
  // which loads from the GOT directly. This avoids run-time overhead
  // at the cost of eager binding.
  auto *F = dyn_cast_or_null<Function>(GV);
  if (F && F->hasFnAttribute(Attribute::NonLazyBind)) {
    return WDC65816II::MO_GOTPCREL;
  }

  // Ensure that we don't emit PLT relocations when in non-pic modes.
  return isPositionIndependent() ? WDC65816II::MO_PLT : WDC65816II::MO_ABSOLUTE_ADDRESS;
}
