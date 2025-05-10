//===-- WDC65816BaseInfo.h - Top level definitions for WDC65816 MC ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains small standalone helper functions and enum definitions
/// for the WDC65816 target useful for the compiler back-end and the MC
/// libraries.  As such, it deliberately does not include references to LLVM
/// core code gen types, passes, etc..
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_WDC65816_MCTARGETDESC_WDC65816BASEINFO_H
#define LLVM_LIB_TARGET_WDC65816_MCTARGETDESC_WDC65816BASEINFO_H

#include "WDC65816MCTargetDesc.h"

#include "llvm/MC/MCExpr.h"
#include "llvm/Support/DataTypes.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/ErrorHandling.h"

#define GET_INSTRINFO_MI_OPS_INFO
#define GET_INSTRINFO_OPERAND_TYPES_ENUM
#define GET_INSTRINFO_LOGICAL_OPERAND_SIZE_MAP
#include "WDC65816GenInstrInfo.inc"

namespace llvm {

namespace WDC65816 {

/// Enums for memory operand decoding. Supports these forms:
/// (d,An)
/// (d,An,Xn)
/// ([bd,An],Xn,od)
/// ([bd,An,Xn],od)
/// TODO Implement scaling other than 1
enum { MemDisp = 0, MemBase = 1, MemIndex = 2, MemOuter = 3 };

/// Enums for pc-relative memory operand decoding. Supports these forms:
/// (d,PC)
/// (d,PC,Xn)
/// ([bd,PC],Xn,od)
/// ([bd,PC,Xn],od)
enum { PCRelDisp = 0, PCRelIndex = 1, PCRelOuter = 2 };

enum class MemAddrModeKind : unsigned {
  j = 1, // (An)
  o,     // (An)+
  e,     // -(An)
  p,     // (d,An)
  f,     // (d,An,Xn.L)
  F,     // (d,An,Xn.W)
  g,     // (d,An,Xn.L,SCALE)
  G,     // (d,An,Xn.W,SCALE)
  u,     // ([bd,An],Xn.L,SCALE,od)
  U,     // ([bd,An],Xn.W,SCALE,od)
  v,     // ([bd,An,Xn.L,SCALE],od)
  V,     // ([bd,An,Xn.W,SCALE],od)
  b,     // abs.L
  B,     // abs.W
  q,     // (d,PC)
  k,     // (d,PC,Xn.L)
  K,     // (d,PC,Xn.W)
  l,     // (d,PC,Xn.L,SCALE)
  L,     // (d,PC,Xn.W,SCALE)
  x,     // ([bd,PC],Xn.L,SCALE,od)
  X,     // ([bd,PC],Xn.W,SCALE,od)
  y,     // ([bd,PC,Xn.L,SCALE],od)
  Y      // ([bd,PC,Xn.W,SCALE],od)
};

// On a LE host:
// MSB                   LSB    MSB                   LSB
// | 0x12 0x34 | 0xAB 0xCD | -> | 0xAB 0xCD | 0x12 0x34 |
// (On a BE host nothing changes)
template <typename value_t> value_t swapWord(value_t Val) {
  const unsigned NumWords = sizeof(Val) / 2;
  if (NumWords <= 1)
    return Val;
  Val = support::endian::byte_swap(Val, llvm::endianness::big);
  value_t NewVal = 0;
  for (unsigned i = 0U; i != NumWords; ++i) {
    uint16_t Part = (Val >> (i * 16)) & 0xFFFF;
    Part = support::endian::byte_swap(Part, llvm::endianness::big);
    NewVal |= (Part << (i * 16));
  }
  return NewVal;
}
} // namespace WDC65816

namespace WDC65816Beads {
enum {
  Ctrl = 0x0,
  Bits1 = 0x1,
  Bits2 = 0x2,
  Bits3 = 0x3,
  Bits4 = 0x4,
  DAReg = 0x5,
  DA = 0x6,
  Reg = 0x7,
  DReg = 0x8,
  Disp8 = 0x9,
  Imm8 = 0xA,
  Imm16 = 0xB,
  Imm32 = 0xC,
  Imm3 = 0xD,
};

// Ctrl payload
enum {
  Term = 0x0,
  Ignore = 0x1,
};
} // namespace WDC65816Beads

/// This namespace holds all of the target specific flags that instruction info
/// tracks.
namespace WDC65816II {
/// Target Operand Flag enum.
enum TOF {

  MO_NO_FLAG,

  /// On a symbol operand this indicates that the immediate is the absolute
  /// address of the symbol.
  MO_ABSOLUTE_ADDRESS,

  /// On a symbol operand this indicates that the immediate is the pc-relative
  /// address of the symbol.
  MO_PC_RELATIVE_ADDRESS,

  /// On a symbol operand this indicates that the immediate is the offset to
  /// the GOT entry for the symbol name from the base of the GOT.
  ///
  ///    name@GOT
  MO_GOT,

  /// On a symbol operand this indicates that the immediate is the offset to
  /// the location of the symbol name from the base of the GOT.
  ///
  ///    name@GOTOFF
  MO_GOTOFF,

  /// On a symbol operand this indicates that the immediate is offset to the
  /// GOT entry for the symbol name from the current code location.
  ///
  ///    name@GOTPCREL
  MO_GOTPCREL,

  /// On a symbol operand this indicates that the immediate is offset to the
  /// PLT entry of symbol name from the current code location.
  ///
  ///    name@PLT
  MO_PLT,

  /// On a symbol operand, this indicates that the immediate is the offset to
  /// the slot in GOT which stores the information for accessing the TLS
  /// variable. This is used when operating in Global Dynamic mode.
  ///    name@TLSGD
  MO_TLSGD,

  /// On a symbol operand, this indicates that the immediate is the offset to
  /// variable within the thread local storage when operating in Local Dynamic
  /// mode.
  ///    name@TLSLD
  MO_TLSLD,

  /// On a symbol operand, this indicates that the immediate is the offset to
  /// the slot in GOT which stores the information for accessing the TLS
  /// variable. This is used when operating in Local Dynamic mode.
  ///    name@TLSLDM
  MO_TLSLDM,

  /// On a symbol operand, this indicates that the immediate is the offset to
  /// the variable within the thread local storage when operating in Initial
  /// Exec mode.
  ///    name@TLSIE
  MO_TLSIE,

  /// On a symbol operand, this indicates that the immediate is the offset to
  /// the variable within in the thread local storage when operating in Local
  /// Exec mode.
  ///    name@TLSLE
  MO_TLSLE,

}; // enum TOF

/// Return true if the specified TargetFlag operand is a reference to a stub
/// for a global, not the global itself.
inline static bool isGlobalStubReference(unsigned char TargetFlag) {
  switch (TargetFlag) {
  default:
    return false;
  case WDC65816II::MO_GOTPCREL: // pc-relative GOT reference.
  case WDC65816II::MO_GOT:      // normal GOT reference.
    return true;
  }
}

/// Return True if the specified GlobalValue is a direct reference for a
/// symbol.
inline static bool isDirectGlobalReference(unsigned char Flag) {
  switch (Flag) {
  default:
    return false;
  case WDC65816II::MO_NO_FLAG:
  case WDC65816II::MO_ABSOLUTE_ADDRESS:
  case WDC65816II::MO_PC_RELATIVE_ADDRESS:
    return true;
  }
}

/// Return true if the specified global value reference is relative to a 32-bit
/// PIC base (WDC65816ISD::GLOBAL_BASE_REG). If this is true, the addressing mode
/// has the PIC base register added in.
inline static bool isGlobalRelativeToPICBase(unsigned char TargetFlag) {
  switch (TargetFlag) {
  default:
    return false;
  case WDC65816II::MO_GOTOFF: // isPICStyleGOT: local global.
  case WDC65816II::MO_GOT:    // isPICStyleGOT: other global.
    return true;
  }
}

/// Return True if the specified GlobalValue requires PC addressing mode.
inline static bool isPCRelGlobalReference(unsigned char Flag) {
  switch (Flag) {
  default:
    return false;
  case WDC65816II::MO_GOTPCREL:
  case WDC65816II::MO_PC_RELATIVE_ADDRESS:
    return true;
  }
}

/// Return True if the Block is referenced using PC
inline static bool isPCRelBlockReference(unsigned char Flag) {
  switch (Flag) {
  default:
    return false;
  case WDC65816II::MO_PC_RELATIVE_ADDRESS:
    return true;
  }
}

static inline bool isAddressRegister(unsigned RegNo) {
  switch (RegNo) {
  case WDC65816::WA0:
  case WDC65816::WA1:
  case WDC65816::WA2:
  case WDC65816::WA3:
  case WDC65816::WA4:
  case WDC65816::WA5:
  case WDC65816::WA6:
  case WDC65816::WSP:
  case WDC65816::A0:
  case WDC65816::A1:
  case WDC65816::A2:
  case WDC65816::A3:
  case WDC65816::A4:
  case WDC65816::A5:
  case WDC65816::A6:
  case WDC65816::SP:
    return true;
  default:
    return false;
  }
}

static inline bool hasMultiMIOperands(unsigned Op, unsigned LogicalOpIdx) {
  return WDC65816::getLogicalOperandSize(Op, LogicalOpIdx) > 1;
}

static inline unsigned getMaskedSpillRegister(unsigned order) {
  switch (order) {
  default:
    return 0;
  case 0:
    return WDC65816::D0;
  case 1:
    return WDC65816::D1;
  case 2:
    return WDC65816::D2;
  case 3:
    return WDC65816::D3;
  case 4:
    return WDC65816::D4;
  case 5:
    return WDC65816::D5;
  case 6:
    return WDC65816::D6;
  case 7:
    return WDC65816::D7;
  case 8:
    return WDC65816::A0;
  case 9:
    return WDC65816::A1;
  case 10:
    return WDC65816::A2;
  case 11:
    return WDC65816::A3;
  case 12:
    return WDC65816::A4;
  case 13:
    return WDC65816::A5;
  case 14:
    return WDC65816::A6;
  case 15:
    return WDC65816::SP;
  }
}

} // namespace WDC65816II

} // namespace llvm

#endif // LLVM_LIB_TARGET_WDC65816_MCTARGETDESC_WDC65816BASEINFO_H
