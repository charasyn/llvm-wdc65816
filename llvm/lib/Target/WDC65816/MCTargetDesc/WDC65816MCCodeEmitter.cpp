//===-- WDC65816MCCodeEmitter.cpp - Convert WDC65816 code emitter -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains defintions for WDC65816 code emitter.
///
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/WDC65816MCCodeEmitter.h"
#include "MCTargetDesc/WDC65816BaseInfo.h"
#include "MCTargetDesc/WDC65816FixupKinds.h"
#include "MCTargetDesc/WDC65816MCTargetDesc.h"

#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/EndianStream.h"
#include "llvm/Support/raw_ostream.h"
#include <type_traits>

using namespace llvm;

#define DEBUG_TYPE "m68k-mccodeemitter"

namespace {
class WDC65816MCCodeEmitter : public MCCodeEmitter {
  WDC65816MCCodeEmitter(const WDC65816MCCodeEmitter &) = delete;
  void operator=(const WDC65816MCCodeEmitter &) = delete;
  const MCInstrInfo &MCII;
  MCContext &Ctx;

  void getBinaryCodeForInstr(const MCInst &MI, SmallVectorImpl<MCFixup> &Fixups,
                             APInt &Inst, APInt &Scratch,
                             const MCSubtargetInfo &STI) const;

  void getMachineOpValue(const MCInst &MI, const MCOperand &Op,
                         unsigned InsertPos, APInt &Value,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const;

  template <unsigned Size>
  void encodeRelocImm(const MCInst &MI, unsigned OpIdx, unsigned InsertPos,
                      APInt &Value, SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const;

  template <unsigned Size>
  void encodePCRelImm(const MCInst &MI, unsigned OpIdx, unsigned InsertPos,
                      APInt &Value, SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const;

  void encodeFPSYSSelect(const MCInst &MI, unsigned OpIdx, unsigned InsertPos,
                         APInt &Value, SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const;

public:
  WDC65816MCCodeEmitter(const MCInstrInfo &mcii, MCContext &ctx)
      : MCII(mcii), Ctx(ctx) {}

  ~WDC65816MCCodeEmitter() override {}

  void encodeInstruction(const MCInst &MI, SmallVectorImpl<char> &CB,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;
};

} // end anonymous namespace

#include "WDC65816GenMCCodeEmitter.inc"

// Select the proper unsigned integer type from a bit size.
template <unsigned Size> struct select_uint_t {
  using type = typename std::conditional<
      Size == 8, uint8_t,
      typename std::conditional<
          Size == 16, uint16_t,
          typename std::conditional<Size == 32, uint32_t,
                                    uint64_t>::type>::type>::type;
};

// Figure out which byte we're at in big endian mode.
template <unsigned Size> static unsigned getBytePosition(unsigned BitPos) {
  if (Size % 16) {
    return static_cast<unsigned>(BitPos / 8 + ((BitPos & 0b1111) < 8 ? 1 : -1));
  } else {
    assert(!(BitPos & 0b1111) && "Not aligned to word boundary?");
    return BitPos / 8;
  }
}

// We need special handlings for relocatable & pc-relative operands that are
// larger than a word.
// A WDC65816 instruction is aligned by word (16 bits). That means, 32-bit
// (& 64-bit) immediate values are separated into hi & lo words and placed
// at lower & higher addresses, respectively. For immediate values that can
// be easily expressed in TG, we explicitly rotate the word ordering like
// this:
// ```
// (ascend (slice "$imm", 31, 16), (slice "$imm", 15, 0))
// ```
// For operands that call into encoder functions, we need to use the `swapWord`
// function to assure the correct word ordering on LE host. Note that
// WDC65816MCCodeEmitter does massage _byte_ ordering of the final encoded
// instruction but it assumes everything aligns on word boundaries. So things
// will go wrong if we don't take care of the _word_ ordering here.
template <unsigned Size>
void WDC65816MCCodeEmitter::encodeRelocImm(const MCInst &MI, unsigned OpIdx,
                                       unsigned InsertPos, APInt &Value,
                                       SmallVectorImpl<MCFixup> &Fixups,
                                       const MCSubtargetInfo &STI) const {
  using value_t = typename select_uint_t<Size>::type;
  const MCOperand &MCO = MI.getOperand(OpIdx);
  if (MCO.isImm()) {
    Value |= WDC65816::swapWord<value_t>(static_cast<value_t>(MCO.getImm()));
  } else if (MCO.isExpr()) {
    const MCExpr *Expr = MCO.getExpr();

    // Absolute address
    int64_t Addr;
    if (Expr->evaluateAsAbsolute(Addr)) {
      Value |= WDC65816::swapWord<value_t>(static_cast<value_t>(Addr));
      return;
    }

    // Relocatable address
    unsigned InsertByte = getBytePosition<Size>(InsertPos);
    Fixups.push_back(MCFixup::create(InsertByte, Expr,
                                     getFixupForSize(Size, /*IsPCRel=*/false),
                                     MI.getLoc()));
  }
}

template <unsigned Size>
void WDC65816MCCodeEmitter::encodePCRelImm(const MCInst &MI, unsigned OpIdx,
                                       unsigned InsertPos, APInt &Value,
                                       SmallVectorImpl<MCFixup> &Fixups,
                                       const MCSubtargetInfo &STI) const {
  const MCOperand &MCO = MI.getOperand(OpIdx);
  if (MCO.isImm()) {
    using value_t = typename select_uint_t<Size>::type;
    Value |= WDC65816::swapWord<value_t>(static_cast<value_t>(MCO.getImm()));
  } else if (MCO.isExpr()) {
    const MCExpr *Expr = MCO.getExpr();
    unsigned InsertByte = getBytePosition<Size>(InsertPos);

    // Special handlings for sizes smaller than a word.
    if (Size < 16) {
      int LabelOffset = 0;
      if (InsertPos < 16)
        // If the patch point is at the first word, PC is pointing at the
        // next word.
        LabelOffset = InsertByte - 2;
      else if (InsertByte % 2)
        // Otherwise the PC is pointing at the first byte of this word.
        // So we need to consider the offset between PC and the fixup byte.
        LabelOffset = 1;

      if (LabelOffset)
        Expr = MCBinaryExpr::createAdd(
            Expr, MCConstantExpr::create(LabelOffset, Ctx), Ctx);
    }

    Fixups.push_back(MCFixup::create(InsertByte, Expr,
                                     getFixupForSize(Size, /*IsPCRel=*/true),
                                     MI.getLoc()));
  }
}

void WDC65816MCCodeEmitter::encodeFPSYSSelect(const MCInst &MI, unsigned OpIdx,
                                          unsigned InsertPos, APInt &Value,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  MCRegister FPSysReg = MI.getOperand(OpIdx).getReg();
  switch (FPSysReg) {
  case WDC65816::FPC:
    Value = 0b100;
    break;
  case WDC65816::FPS:
    Value = 0b010;
    break;
  case WDC65816::FPIAR:
    Value = 0b001;
    break;
  default:
    llvm_unreachable("Unrecognized FPSYS register");
  }
}

void WDC65816MCCodeEmitter::getMachineOpValue(const MCInst &MI, const MCOperand &Op,
                                          unsigned InsertPos, APInt &Value,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  // Register
  if (Op.isReg()) {
    unsigned RegNum = Op.getReg();
    const auto *RI = Ctx.getRegisterInfo();
    Value |= RI->getEncodingValue(RegNum);
    // Setup the D/A bit
    if (WDC65816II::isAddressRegister(RegNum))
      Value |= 0b1000;
  } else if (Op.isImm()) {
    // Immediate
    Value |= static_cast<uint64_t>(Op.getImm());
  } else if (Op.isExpr()) {
    // Absolute address
    int64_t Addr;
    if (!Op.getExpr()->evaluateAsAbsolute(Addr))
      report_fatal_error("Unsupported asm expression. Only absolute address "
                         "can be placed here.");
    Value |= static_cast<uint64_t>(Addr);
  } else {
    llvm_unreachable("Unsupported operand type");
  }
}

void WDC65816MCCodeEmitter::encodeInstruction(const MCInst &MI,
                                          SmallVectorImpl<char> &CB,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  LLVM_DEBUG(dbgs() << "EncodeInstruction: " << MCII.getName(MI.getOpcode())
                    << "(" << MI.getOpcode() << ")\n");
  (void)MCII;

  // Try using the new method first.
  APInt EncodedInst(16, 0U);
  APInt Scratch(64, 0U); // One APInt word is enough.
  getBinaryCodeForInstr(MI, Fixups, EncodedInst, Scratch, STI);

  ArrayRef<uint64_t> Data(EncodedInst.getRawData(), EncodedInst.getNumWords());
  int64_t InstSize = EncodedInst.getBitWidth();
  for (uint64_t Word : Data) {
    for (int i = 0; i < 4 && InstSize > 0; ++i, InstSize -= 16) {
      support::endian::write<uint16_t>(CB, static_cast<uint16_t>(Word),
                                       llvm::endianness::big);
      Word >>= 16;
    }
  }
}

MCCodeEmitter *llvm::createWDC65816MCCodeEmitter(const MCInstrInfo &MCII,
                                             MCContext &Ctx) {
  return new WDC65816MCCodeEmitter(MCII, Ctx);
}
