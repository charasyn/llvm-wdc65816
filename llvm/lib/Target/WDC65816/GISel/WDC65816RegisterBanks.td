//===-- WDC65816RegisterBanks.td - Describe the WDC65816 Banks -----*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Define the WDC65816 register banks used for GlobalISel.
///
//===----------------------------------------------------------------------===//

/// General Purpose Registers. Here we define a register bank with name AnyGPR
def GPRRegBank : RegisterBank<"AnyGPR", [XR32]>;
