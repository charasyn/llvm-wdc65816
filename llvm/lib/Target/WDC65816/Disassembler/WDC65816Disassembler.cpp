//===-- WDC65816Disassembler.cpp - Disassembler for WDC65816 ------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is part of the WDC65816 Disassembler.
//
//===----------------------------------------------------------------------===//

#include "WDC65816.h"
#include "WDC65816RegisterInfo.h"
#include "WDC65816Subtarget.h"
#include "MCTargetDesc/WDC65816MCCodeEmitter.h"
#include "MCTargetDesc/WDC65816MCTargetDesc.h"
#include "TargetInfo/WDC65816TargetInfo.h"

#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCDecoderOps.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "m68k-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

static const unsigned RegisterDecode[] = {
    WDC65816::D0,    WDC65816::D1,  WDC65816::D2,  WDC65816::D3,  WDC65816::D4,  WDC65816::D5,
    WDC65816::D6,    WDC65816::D7,  WDC65816::A0,  WDC65816::A1,  WDC65816::A2,  WDC65816::A3,
    WDC65816::A4,    WDC65816::A5,  WDC65816::A6,  WDC65816::SP,  WDC65816::FP0, WDC65816::FP1,
    WDC65816::FP2,   WDC65816::FP3, WDC65816::FP4, WDC65816::FP5, WDC65816::FP6, WDC65816::FP7,
    WDC65816::FPIAR, WDC65816::FPS, WDC65816::FPC};

static DecodeStatus DecodeRegisterClass(MCInst &Inst, uint64_t RegNo,
                                        uint64_t Address, const void *Decoder) {
  if (RegNo >= 24)
    return DecodeStatus::Fail;
  Inst.addOperand(MCOperand::createReg(RegisterDecode[RegNo]));
  return DecodeStatus::Success;
}

static DecodeStatus DecodeDR32RegisterClass(MCInst &Inst, uint64_t RegNo,
                                            uint64_t Address,
                                            const void *Decoder) {
  return DecodeRegisterClass(Inst, RegNo, Address, Decoder);
}

static DecodeStatus DecodeDR16RegisterClass(MCInst &Inst, uint64_t RegNo,
                                            uint64_t Address,
                                            const void *Decoder) {
  return DecodeRegisterClass(Inst, RegNo, Address, Decoder);
}

static DecodeStatus DecodeDR8RegisterClass(MCInst &Inst, uint64_t RegNo,
                                           uint64_t Address,
                                           const void *Decoder) {
  return DecodeRegisterClass(Inst, RegNo, Address, Decoder);
}

static DecodeStatus DecodeAR32RegisterClass(MCInst &Inst, uint64_t RegNo,
                                            uint64_t Address,
                                            const void *Decoder) {
  return DecodeRegisterClass(Inst, RegNo | 8ULL, Address, Decoder);
}

static DecodeStatus DecodeAR16RegisterClass(MCInst &Inst, uint64_t RegNo,
                                            uint64_t Address,
                                            const void *Decoder) {
  return DecodeRegisterClass(Inst, RegNo | 8ULL, Address, Decoder);
}

static DecodeStatus DecodeXR32RegisterClass(MCInst &Inst, uint64_t RegNo,
                                            uint64_t Address,
                                            const void *Decoder) {
  return DecodeRegisterClass(Inst, RegNo, Address, Decoder);
}

static DecodeStatus DecodeXR16RegisterClass(MCInst &Inst, uint64_t RegNo,
                                            uint64_t Address,
                                            const void *Decoder) {
  return DecodeRegisterClass(Inst, RegNo, Address, Decoder);
}

static DecodeStatus DecodeFPDRRegisterClass(MCInst &Inst, uint64_t RegNo,
                                            uint64_t Address,
                                            const void *Decoder) {
  return DecodeRegisterClass(Inst, RegNo | 16ULL, Address, Decoder);
}
#define DecodeFPDR32RegisterClass DecodeFPDRRegisterClass
#define DecodeFPDR64RegisterClass DecodeFPDRRegisterClass
#define DecodeFPDR80RegisterClass DecodeFPDRRegisterClass

static DecodeStatus DecodeFPCSCRegisterClass(MCInst &Inst, uint64_t RegNo,
                                             uint64_t Address,
                                             const void *Decoder) {
  return DecodeRegisterClass(Inst, (RegNo >> 1) + 24, Address, Decoder);
}
#define DecodeFPICRegisterClass DecodeFPCSCRegisterClass

static DecodeStatus DecodeCCRCRegisterClass(MCInst &Inst, APInt &Insn,
                                            uint64_t Address,
                                            const void *Decoder) {
  llvm_unreachable("unimplemented");
}

static DecodeStatus DecodeImm32(MCInst &Inst, uint64_t Imm, uint64_t Address,
                                const void *Decoder) {
  Inst.addOperand(MCOperand::createImm(WDC65816::swapWord<uint32_t>(Imm)));
  return DecodeStatus::Success;
}

#include "WDC65816GenDisassemblerTable.inc"

#undef DecodeFPDR32RegisterClass
#undef DecodeFPDR64RegisterClass
#undef DecodeFPDR80RegisterClass
#undef DecodeFPICRegisterClass

/// A disassembler class for WDC65816.
struct WDC65816Disassembler : public MCDisassembler {
  WDC65816Disassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx) {}
  virtual ~WDC65816Disassembler() {}

  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &CStream) const override;
};

DecodeStatus WDC65816Disassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                              ArrayRef<uint8_t> Bytes,
                                              uint64_t Address,
                                              raw_ostream &CStream) const {
  DecodeStatus Result;
  auto MakeUp = [&](APInt &Insn, unsigned InstrBits) {
    unsigned Idx = Insn.getBitWidth() >> 3;
    unsigned RoundUp = alignTo(InstrBits, Align(16));
    if (RoundUp > Insn.getBitWidth())
      Insn = Insn.zext(RoundUp);
    RoundUp = RoundUp >> 3;
    for (; Idx < RoundUp; Idx += 2) {
      Insn.insertBits(support::endian::read16be(&Bytes[Idx]), Idx * 8, 16);
    }
  };
  APInt Insn(16, support::endian::read16be(Bytes.data()));
  // 2 bytes of data are consumed, so set Size to 2
  // If we don't do this, disassembler may generate result even
  // the encoding is invalid. We need to let it fail correctly.
  Size = 2;
  Result = decodeInstruction(DecoderTable80, Instr, Insn, Address, this, STI,
                             MakeUp);
  if (Result == DecodeStatus::Success)
    Size = InstrLenTable[Instr.getOpcode()] >> 3;
  return Result;
}

static MCDisassembler *createWDC65816Disassembler(const Target &T,
                                              const MCSubtargetInfo &STI,
                                              MCContext &Ctx) {
  return new WDC65816Disassembler(STI, Ctx);
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeWDC65816Disassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getTheWDC65816Target(),
                                         createWDC65816Disassembler);
}
