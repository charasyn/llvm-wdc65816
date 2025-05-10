//===-- WDC65816Schedule.td - WDC65816 Scheduling Definitions ------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains WDC65816 scheduler definitions.
///
//===----------------------------------------------------------------------===//

/// This is a very general WDC65816 Scheduling Model and best suited for the very
/// first M68000 CPU, other model must override these characteristics
class WDC65816SchedModel : SchedMachineModel {
  let LoadLatency = 4;  // Word (Rn)
  let HighLatency = 16; // Long ABS
  let PostRAScheduler = 0;
  let CompleteModel = 0;
}

def GenericWDC65816Model : WDC65816SchedModel;
