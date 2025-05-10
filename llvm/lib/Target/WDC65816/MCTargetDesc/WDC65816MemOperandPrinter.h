//===-- WDC65816MemOperandPrinter.h - Memory operands printing ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains memory operand printing logics shared between AsmPrinter
//  and MCInstPrinter.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_WDC65816_MEMOPERANDPRINTER_WDC65816INSTPRINTER_H
#define LLVM_LIB_TARGET_WDC65816_MEMOPERANDPRINTER_WDC65816INSTPRINTER_H

#include "WDC65816BaseInfo.h"

#include "llvm/Support/raw_ostream.h"

namespace llvm {
template <class Derived, typename InstTy> class WDC65816MemOperandPrinter {
  Derived &impl() { return *static_cast<Derived *>(this); }

protected:
  void printARIMem(const InstTy *MI, unsigned OpNum, raw_ostream &O) {
    O << '(';
    impl().printOperand(MI, OpNum, O);
    O << ')';
  }

  void printARIPIMem(const InstTy *MI, unsigned OpNum, raw_ostream &O) {
    O << "(";
    impl().printOperand(MI, OpNum, O);
    O << ")+";
  }

  void printARIPDMem(const InstTy *MI, unsigned OpNum, raw_ostream &O) {
    O << "-(";
    impl().printOperand(MI, OpNum, O);
    O << ")";
  }

  void printARIDMem(const InstTy *MI, unsigned OpNum, raw_ostream &O) {
    O << '(';
    impl().printDisp(MI, OpNum + WDC65816::MemDisp, O);
    O << ',';
    impl().printOperand(MI, OpNum + WDC65816::MemBase, O);
    O << ')';
  }

  void printARIIMem(const InstTy *MI, unsigned OpNum, raw_ostream &O) {
    O << '(';
    impl().printDisp(MI, OpNum + WDC65816::MemDisp, O);
    O << ',';
    impl().printOperand(MI, OpNum + WDC65816::MemBase, O);
    O << ',';
    impl().printOperand(MI, OpNum + WDC65816::MemIndex, O);
    O << ')';
  }

  void printPCDMem(const InstTy *MI, uint64_t Address, unsigned OpNum,
                   raw_ostream &O) {
    O << '(';
    impl().printDisp(MI, OpNum + WDC65816::PCRelDisp, O);
    O << ",%pc)";
  }

  void printPCIMem(const InstTy *MI, uint64_t Address, unsigned OpNum,
                   raw_ostream &O) {
    O << '(';
    impl().printDisp(MI, OpNum + WDC65816::PCRelDisp, O);
    O << ",%pc,";
    impl().printOperand(MI, OpNum + WDC65816::PCRelIndex, O);
    O << ')';
  }
};
} // end namespace llvm
#endif
