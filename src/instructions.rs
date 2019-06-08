use super::*;

impl W65C02S {
    pub(crate) fn brk<S: System>(&mut self, system: &mut S) {
        let pc = self.read_pc_postincrement();
        system.read_operand_spurious(self, pc);
        let pc = self.get_pc();
        self.push(system, (pc >> 8) as u8);
        self.push(system, pc as u8);
        self.push(system, self.p | P_B);
        self.p &= !P_D;
        self.p |= P_I;
        self.pc = (self.pc & 0xFF00) | (system.read_vector(self, IRQ_VECTOR) as u16);
        self.pc = (self.pc & 0x00FF) | (system.read_vector(self, IRQ_VECTOR+1) as u16) << 8;
    }
    pub(crate) fn jsr<S: System>(&mut self, system: &mut S) {
        let pc = self.read_pc_postincrement();
        let target_lo = system.read_operand(self, pc);
        self.spurious_stack_read(system);
        self.push(system, (self.pc >> 8) as u8);
        self.push(system, self.pc as u8);
        self.check_irq_edge();
        let target_hi = system.read_operand(self, self.pc);
        self.pc = (target_hi as u16) << 8 | (target_lo as u16);
    }
    pub(crate) fn rts<S: System>(&mut self, system: &mut S) {
        let pc = self.get_pc();
        system.read_operand_spurious(self, pc);
        self.spurious_stack_read(system);
        self.pc = (self.pc & 0xFF00) | self.pop(system) as u16;
        self.pc = (self.pc & 0x00FF) | (self.pop(system) as u16) << 8;
        self.check_irq_edge();
        system.read_operand_spurious(self, self.pc);
        self.pc = self.pc.wrapping_add(1);
    }
    pub(crate) fn rti<S: System>(&mut self, system: &mut S) {
        let pc = self.get_pc();
        system.read_operand_spurious(self, pc);
        self.spurious_stack_read(system);
        let new_p = self.pop(system);
        self.set_p(new_p);
        self.pc = (self.pc & 0xFF00) | self.pop(system) as u16;
        self.check_irq_edge();
        self.pc = (self.pc & 0x00FF) | (self.pop(system) as u16) << 8;
    }
    pub(crate) fn jmp<R: HasEA, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let am = AM::get_operand(system, self);
        self.check_irq_edge();
        self.pc = am.get_effective_address();
    }
    pub(crate) fn sta<R: Writable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        am.write(system, self, self.a)
    }
    pub(crate) fn stx<R: Writable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        am.write(system, self, self.x)
    }
    pub(crate) fn sty<R: Writable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        am.write(system, self, self.y)
    }
    pub(crate) fn stz<R: Writable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        am.write(system, self, 0)
    }
    pub(crate) fn lda<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        self.a = am.read(system, self);
        self.nz_p(self.a);
    }
    pub(crate) fn ldx<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        self.x = am.read(system, self);
        self.nz_p(self.x);
    }
    pub(crate) fn ldy<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        self.y = am.read(system, self);
        self.nz_p(self.y);
    }
    pub(crate) fn ora<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        self.check_irq_edge();
        self.a |= AM::get_operand(system, self).read(system, self);
        self.nz_p(self.a)
    }
    pub(crate) fn and<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        self.check_irq_edge();
        self.a &= AM::get_operand(system, self).read(system, self);
        self.nz_p(self.a)
    }
    pub(crate) fn bit<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        self.check_irq_edge();
        let data = AM::get_operand(system, self).read(system, self);
        if data & self.a == 0 { self.p = self.p | P_Z }
        else { self.p = self.p & !P_Z }
        self.p = (self.p & 0x3F) | (data & 0xC0);
    }
    pub(crate) fn bit_i<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        self.check_irq_edge();
        let data = AM::get_operand(system, self).read(system, self);
        if data & self.a == 0 { self.p = self.p | P_Z }
        else { self.p = self.p & !P_Z }
    }
    pub(crate) fn eor<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        self.check_irq_edge();
        self.a ^= AM::get_operand(system, self).read(system, self);
        self.nz_p(self.a)
    }
    pub(crate) fn nop<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        am.read_spurious(system, self);
    }
    // $5C is an especially weird one
    pub(crate) fn nop_5c<R: HasEA, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let am = AM::get_operand(system, self);
        self.check_irq_edge();
        system.read_spurious(self, am.get_effective_address() | 0xFF00);
        system.read_spurious(self, 0xFFFF);
        system.read_spurious(self, 0xFFFF);
        system.read_spurious(self, 0xFFFF);
        self.check_irq_edge();
        system.read_spurious(self, 0xFFFF);
    }
    pub(crate) fn trb<R: RMWable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        let data = am.read(system, self);
        am.read_locked_spurious(system, self);
        self.check_irq_edge();
        am.write_locked(system, self, data & !self.a);
        if data & self.a != 0 { self.p &= !P_Z }
        else { self.p |= P_Z }
    }
    pub(crate) fn tsb<R: RMWable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        let data = am.read(system, self);
        am.read_locked_spurious(system, self);
        self.check_irq_edge();
        am.write_locked(system, self, data | self.a);
        if data & self.a != 0 { self.p &= !P_Z }
        else { self.p |= P_Z }
    }
    pub(crate) fn asl<R: RMWable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        let data = am.read(system, self);
        am.read_locked_spurious(system, self);
        let result = data << 1;
        self.check_irq_edge();
        am.write_locked(system, self, result);
        self.cnz_p(data & 0x80 != 0, result);
    }
    pub(crate) fn lsr<R: RMWable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        let data = am.read(system, self);
        am.read_locked_spurious(system, self);
        let result = data >> 1;
        self.check_irq_edge();
        am.write_locked(system, self, result);
        self.cnz_p(data & 0x01 != 0, result);
    }
    pub(crate) fn rol<R: RMWable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        let data = am.read(system, self);
        am.read_locked_spurious(system, self);
        let result = data << 1 | if self.p & P_C != 0 { 1 } else { 0 };
        self.check_irq_edge();
        am.write_locked(system, self, result);
        self.cnz_p(data & 0x80 != 0, result);
    }
    pub(crate) fn ror<R: RMWable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        let data = am.read(system, self);
        am.read_locked_spurious(system, self);
        let result = data >> 1 | if self.p & P_C != 0 { 0x80 } else { 0 };
        self.check_irq_edge();
        am.write_locked(system, self, result);
        self.cnz_p(data & 0x01 != 0, result);
    }
    pub(crate) fn inc<R: RMWable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        let data = am.read(system, self);
        am.read_locked_spurious(system, self);
        self.check_irq_edge();
        let result = data.wrapping_add(1);
        am.write_locked(system, self, result);
        self.nz_p(result);
    }
    pub(crate) fn dec<R: RMWable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        let data = am.read(system, self);
        am.read_locked_spurious(system, self);
        self.check_irq_edge();
        let result = data.wrapping_sub(1);
        am.write_locked(system, self, result);
        self.nz_p(result);
    }
    // note that unlike the other RMW instructions, RMBx/SMBx have THREE locked
    // cycles, not two.
    pub(crate) fn rmb<R: RMWable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S, mask: u8) {
        let mut am = AM::get_operand(system, self);
        let data = am.read_locked(system, self);
        am.read_locked_spurious(system, self);
        let result = data & mask;
        self.check_irq_edge();
        am.write_locked(system, self, result);
    }
    pub(crate) fn smb<R: RMWable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S, mask: u8) {
        let mut am = AM::get_operand(system, self);
        let data = am.read_locked(system, self);
        am.read_locked_spurious(system, self);
        let result = data | mask;
        self.check_irq_edge();
        am.write_locked(system, self, result);
    }
    pub(crate) fn branch<R: Branchable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S, should_branch: bool) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        if should_branch {
            self.pc = am.get_branch_target(system, self);
        }
    }
    pub(crate) fn bbr<R: Readable + Branchable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S, mask: u8) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        if am.read(system, self) & mask == 0 {
            self.pc = am.get_branch_target(system, self);
        }
    }
    pub(crate) fn bbs<R: Readable + Branchable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S, mask: u8) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        if am.read(system, self) & mask == mask {
            self.pc = am.get_branch_target(system, self);
        }
    }
    pub(crate) fn stp<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.state = State::Stopped;
    }
    pub(crate) fn wai<S: System>(&mut self, _: &mut S) {
        self.state = State::AwaitingInterrupt;
    }
    pub(crate) fn clc<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.p &= !P_C;
    }
    pub(crate) fn sec<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.p |= P_C;
    }
    pub(crate) fn clv<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.p &= !P_V;
    }
    pub(crate) fn cld<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.p &= !P_D;
    }
    pub(crate) fn sed<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.p |= P_D;
    }
    pub(crate) fn cli<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.p &= !P_I;
    }
    pub(crate) fn sei<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.p |= P_I;
    }
    pub(crate) fn php<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.check_irq_edge();
        self.push(system, self.p | P_B | P_1);
    }
    pub(crate) fn plp<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.spurious_stack_read(system);
        self.check_irq_edge();
        let new_p = self.pop(system);
        self.set_p(new_p);
    }
    pub(crate) fn pha<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.check_irq_edge();
        self.push(system, self.a);
    }
    pub(crate) fn pla<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.spurious_stack_read(system);
        self.check_irq_edge();
        self.a = self.pop(system);
    }
    pub(crate) fn phx<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.check_irq_edge();
        self.push(system, self.x);
    }
    pub(crate) fn plx<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.spurious_stack_read(system);
        self.check_irq_edge();
        self.x = self.pop(system);
    }
    pub(crate) fn phy<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.check_irq_edge();
        self.push(system, self.y);
    }
    pub(crate) fn ply<S: System>(&mut self, system: &mut S) {
        system.read_operand_spurious(self, self.pc);
        self.spurious_stack_read(system);
        self.check_irq_edge();
        self.y = self.pop(system);
    }
    pub(crate) fn tax<S: System>(&mut self, system: &mut S) {
        self.check_irq_edge();
        system.read_operand_spurious(self, self.pc);
        self.x = self.a;
        self.nz_p(self.x);
    }
    pub(crate) fn tay<S: System>(&mut self, system: &mut S) {
        self.check_irq_edge();
        system.read_operand_spurious(self, self.pc);
        self.y = self.a;
        self.nz_p(self.y);
    }
    pub(crate) fn txa<S: System>(&mut self, system: &mut S) {
        self.check_irq_edge();
        system.read_operand_spurious(self, self.pc);
        self.a = self.x;
        self.nz_p(self.a);
    }
    pub(crate) fn tya<S: System>(&mut self, system: &mut S) {
        self.check_irq_edge();
        system.read_operand_spurious(self, self.pc);
        self.a = self.y;
        self.nz_p(self.a);
    }
    pub(crate) fn txs<S: System>(&mut self, system: &mut S) {
        self.check_irq_edge();
        system.read_operand_spurious(self, self.pc);
        self.s = self.x;
    }
    pub(crate) fn tsx<S: System>(&mut self, system: &mut S) {
        self.check_irq_edge();
        system.read_operand_spurious(self, self.pc);
        self.x = self.s;
        self.nz_p(self.x);
    }
    pub(crate) fn adc<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        let red = am.read(system, self);
        let val = if (self.p & P_D) != 0 {
            self.check_irq_edge();
            am.read_spurious(system, self);
            let mut al = (self.a & 0xF) + (red & 0xF) + if (self.p & P_C) != 0 { 1 } else { 0 };
            if al > 9 { al = ((al + 6) & 0xF) | 0x10 }
            let val = ((self.a as i8 as u16) & 0xFFF0) + ((red as i8 as u16) & 0xFFF0) + al as u16;
            if val >= 0x80 && val < 0xFF80 { self.p |= P_V }
            else { self.p &= !P_V }
            // *facepalm*
            let val = (self.a as u16 & 0xF0) + (red as u16 & 0xF0) + al as u16;
            if val > 0x9F { (val + 0x60) | 0x100 } else { val }
        }
        else {
            let mut val = self.a as u16 + red as u16;
            if (self.p & P_C) != 0 { val += 1 }
            if ((self.a ^ (val as u8)) & (red ^ (val as u8)) & 0x80) != 0 { self.p |= P_V }
            else { self.p &= !P_V }
            val
        };
        self.a = val as u8;
        self.cnz_p(val >= 0x0100, val as u8);
    }
    pub(crate) fn sbc<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        let red = am.read(system, self);
        let val = if (self.p & P_D) != 0 {
            self.check_irq_edge();
            am.read_spurious(system, self);
            let al = (self.a & 0xF).wrapping_sub(red & 0xF).wrapping_sub(if (self.p & P_C) != 0 { 0 } else { 1 });
            let mut val = (self.a as u16).wrapping_sub(red as u16).wrapping_sub(if (self.p & P_C) != 0 { 0 } else { 1 });
            if ((self.a as u16 ^ val) & (red as u16 ^ 0xFF ^ val) & 0x80) != 0 { self.p |= P_V }
            else { self.p &= !P_V }
            if (val & 0x8000) != 0 {
                val -= 0x60;
                self.p &= !P_C;
            }
            else {
                self.p |= P_C;
            }
            if al >= 0x80 { val -= 0x06 }
            self.nz_p(val as u8);
            val
        }
        else {
            let red = red ^ 0xFF;
            let mut val = self.a as u16 + red as u16;
            if (self.p & P_C) != 0 { val += 1 }
            if ((self.a ^ (val as u8)) & (red ^ (val as u8)) & 0x80) != 0 { self.p |= P_V }
            else { self.p &= !P_V }
            self.cnz_p(val >= 0x0100, val as u8);
            val
        };
        self.a = val as u8;
    }
    pub(crate) fn cmp<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        let red = am.read(system, self);
        let val = (self.a as u16) + (red as u16 ^ 0xFF) + 1;
        self.cnz_p(val >= 0x0100, val as u8);
    }
    pub(crate) fn cpx<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        let red = am.read(system, self);
        let val = (self.x as u16) + (red as u16 ^ 0xFF) + 1;
        self.cnz_p(val >= 0x0100, val as u8);
    }
    pub(crate) fn cpy<R: Readable, AM: AddressingMode<Result = R>, S: System>(&mut self, system: &mut S) {
        let mut am = AM::get_operand(system, self);
        self.check_irq_edge();
        let red = am.read(system, self);
        let val = (self.y as u16) + (red as u16 ^ 0xFF) + 1;
        self.cnz_p(val >= 0x0100, val as u8);
    }
}
