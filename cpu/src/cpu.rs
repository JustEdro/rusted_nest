use super::opcodes::{AddressingMode, CPU_OPS_CODES};

/*
Accessing only registers	2
Accessing the first 255 bytes of RAM	3
Accessing memory space after the first 255	4-7

Memory space [0x0100 .. 0x1FF] is used for stack. The stack pointer holds the address of the top of that space. NES Stack (as all stacks) grows from top to bottom: when a byte gets pushed to the stack, SP register decrements
*/

// https://www.nesdev.org/obelisk-6502-guide/reference.html
// http://www.6502.org/tutorials/6502opcodes.html
pub struct Cpu6502 {
    program_counter: u16, // Program Counter (PC)
    stack_pointer: u8,    // Stack Pointer
    register_acc: u8,     // Accumulator (A)
    register_x: u8,       // Index Register X (X)
    register_y: u8,       // Index Register Y (Y)
    register_status: u8,  // Processor status (P)
    memory: [u8; 0xFFFF],
}

impl Cpu6502 {
    pub fn new() -> Self {
        Cpu6502 {
            program_counter: 0,
            stack_pointer: 0,
            register_acc: 0,
            register_x: 0,
            register_y: 0,
            register_status: 0,
            memory: [0; 0xFFFF],
        }
    }

    fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    fn mem_read_u16(&self, pos: u16) -> u16 {
        let lo = self.mem_read(pos) as u16;
        let hi = self.mem_read(pos + 1) as u16;
        (hi << 8) | (lo as u16)
    }

    fn mem_write_u16(&mut self, pos: u16, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.mem_write(pos, lo);
        self.mem_write(pos + 1, hi);
    }

    fn push(&mut self, data: u8){
        let addr = 0x0100 | self.stack_pointer as u16;
        self.mem_write(addr, data);
        self.stack_pointer -= 1;
    }

    fn push_u16(&mut self, data: u16){
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.push(hi);
        self.push(lo);
    }

    fn pull(&mut self) -> u8 {
        self.stack_pointer += 1;
        let addr = 0x0100 | self.stack_pointer as u16;
        self.mem_read(addr)
    }

    fn pull_u16(&mut self) -> u16 {
        let lo = self.pull() as u16;
        let hi = self.pull() as u16;
        (hi << 8) | (lo as u16)
    }


    // TODO pop

    pub fn reset(&mut self) {
        self.register_acc = 0;
        self.register_x = 0;
        self.register_status = 0;
        self.stack_pointer = 0xFF; // Memory space [0x0100 .. 0x01FF] is used for stack.

        self.program_counter = self.mem_read_u16(0xFFFC);
    }

    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000..(0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_u16(0xFFFC, 0x8000);
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run()
    }

    #[inline]
    fn get_operand(&self, mode: &AddressingMode) -> u8 {
        match mode {
            AddressingMode::Accumulator => self.register_acc,
            _ => {
                let addr = self.get_operand_address(mode);
                self.mem_read(addr)
            }
        }
    }

    #[inline]
    fn store_operand(&mut self, mode: &AddressingMode, value: u8) {
        match mode {
            AddressingMode::Accumulator => {
                self.register_acc = value;
            }
            _ => {
                let addr = self.get_operand_address(mode);
                self.mem_write(addr, value);
            }
        }
    }

    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        let operand_pc = self.program_counter + 1;
        match mode {
            AddressingMode::Immediate | AddressingMode::Relative => operand_pc,

            AddressingMode::ZeroPage => self.mem_read(operand_pc) as u16,

            AddressingMode::Absolute => self.mem_read_u16(operand_pc),

            AddressingMode::Indirect => {
                let addr = self.mem_read_u16(operand_pc);
                self.mem_read_u16(addr)

                /*
                An original 6502 has does not correctly fetch the target address 
                if the indirect vector falls on a page boundary 
                (e.g. $xxFF where xx is any value from $00 to $FF). 
                In this case fetches the LSB from $xxFF as expected but takes the MSB from $xx00. 
                This is fixed in some later chips like the 65SC02 so for compatibility
                always ensure the indirect vector is not at the end of the page. 
                */
            }

            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read(operand_pc);
                let addr = pos.wrapping_add(self.register_x) as u16;
                addr
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read(operand_pc);
                let addr = pos.wrapping_add(self.register_y) as u16;
                addr
            }

            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(operand_pc);
                let addr = base.wrapping_add(self.register_x as u16);
                addr
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(operand_pc);
                let addr = base.wrapping_add(self.register_y as u16);
                addr
            }

            AddressingMode::Indirect_X => {
                let base = self.mem_read(operand_pc);

                let ptr: u8 = (base as u8).wrapping_add(self.register_x);
                let lo = self.mem_read(ptr as u16);
                let hi = self.mem_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(operand_pc);

                let lo = self.mem_read(base as u16);
                let hi = self.mem_read((base as u8).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.register_y as u16);
                deref
            }

            AddressingMode::NoneAddressing | AddressingMode::Accumulator => {
                panic!("mode {:?} is not supported", mode);
            }
        }
    }

    pub fn run(&mut self) {
        loop {
            let opscode = self.mem_read(self.program_counter) as usize;
            let op = CPU_OPS_CODES[opscode];

            if op.opcode != opscode as u8 {
                panic!("Opcode ${:02x} is not implemented", opscode)
            }

            let am = &op.address_mode;

            // store PC to check for a jump
            let pc = self.program_counter;

            // pick a function
            // TODO there should be a better way of doing that
            match opscode {
                // ADC
                0x69 | 0x65 | 0x75 | 0x6D | 0x7D | 0x79 | 0x61 | 0x71 => {
                    self.adc(am);
                }

                // AND
                0x29 | 0x25 | 0x35 | 0x2D | 0x3D | 0x39 | 0x21 | 0x31 => {
                    self.and(am);
                }

                // ASL
                0x0A | 0x06 | 0x16 | 0x0E | 0x1E => {
                    self.asl(am);
                }

                // BCC
                0x90 => {
                    self.bcc(am);
                }

                // BCS
                0xB0 => {
                    self.bcs(am);
                }

                // BEQ
                0xF0 => {
                    self.beq(am);
                }

                // BMI
                0x30 => {
                    self.bmi(am);
                }

                // BNE
                0xD0 => {
                    self.bne(am);
                }

                // BPL
                0x10 => {
                    self.bpl(am);
                }

                // BVC
                0x50 => {
                    self.bvc(am);
                }

                // BVS
                0x70 => {
                    self.bvs(am);
                }

                // BIT
                0x24 | 0x2C => {
                    self.bit(am);
                }

                // BRK
                0x00 => {
                    return;
                }

                // CLC
                0x18 => {
                    self.clc();
                }

                // CLD
                0xD8 => {
                    // The state of the decimal flag is uncertain when the CPU is powered up and it is 
                    // not reset when an interrupt is generated. In both cases you should include an 
                    // explicit CLD to ensure that the flag is cleared before performing addition or subtraction.
                    self.cld();
                }

                // CLI
                0x58 => {
                    self.cli();
                }

                // CLV
                0xB8 => {
                    self.clv();
                }

                // CMP
                0xC9 | 0xC5 | 0xD5 | 0xCD | 0xDD | 0xD9 | 0xC1 | 0xD1 => {
                    self.cmp(am);
                }

                // CPX
                0xE0 | 0xE4 | 0xEC => {
                    self.cpx(am);
                }

                // CPY
                0xC0 | 0xC4 | 0xCC => {
                    self.cpy(am);
                }

                // DEC
                0xC6 | 0xD6 | 0xCE | 0xDE => {
                    self.dec(am);
                }

                // DEX
                0xCA => {
                    self.dex();
                }

                // DEY
                0x88 => {
                    self.dey();
                }

                // EOR
                0x49 | 0x45 | 0x55 | 0x4D | 0x5D | 0x59 | 0x41 | 0x51  => {
                    self.eor(am);
                }

                // INC
                0xE6 | 0xF6 | 0xEE | 0xFE => {
                    self.inc(am);
                }

                // INX
                0xE8 => {
                    self.inx();
                }

                // INY
                0xC8 => {
                    self.iny();
                }

                // JMP
                0x4C | 0x6C => {
                    self.jmp(am);
                }

                // JSR
                0x20 => {
                    self.jsr(am);
                }

                // LDA
                0xA9 | 0xA5 | 0xB5 | 0xAD | 0xBD | 0xB9 | 0xA1 | 0xB1 => {
                    self.lda(am);
                }

                // LDX
                0xA2 | 0xA6 | 0xB6 | 0xAE | 0xBE => {
                    self.ldx(am);
                }

                // LDY
                0xA0 | 0xA4 | 0xB4 | 0xAC | 0xBC => {
                    self.ldy(am);
                }

                // LSR
                0x4A | 0x46 | 0x56 | 0x4E | 0x5E => {
                    self.lsr(am);
                }

                // NOP
                0xEA => {
                    self.nop();
                }

                // ORA
                0x09 | 0x05 | 0x15 | 0x0D | 0x1D | 0x19 | 0x01 | 0x11 => {
                    self.ora(am);
                }

                // PHA
                0x48 => {
                    self.pha();
                }

                // PHP
                0x08 => {
                    self.php();
                }

                // PLA
                0x68 => {
                    self.pla();
                }

                // PLP
                0x28 => {
                    self.plp();
                }

                // ROL
                0x2A | 0x26 | 0x36 | 0x2E | 0x3E => {
                    self.rol(am);
                }

                // ROR
                0x6A | 0x66 | 0x76 | 0x6E | 0x7E => {
                    self.ror(am);
                }

                // RTI
                0x40 => todo!(),

                // RTS
                0x60 => {
                    self.rts();
                }

                // SBC
                0xE9 | 0xE5 | 0xF5 | 0xED | 0xFD | 0xF9 | 0xE1 | 0xF1 => {
                    self.sbc(am);
                }
                
                // SEC
                0x38 => {
                    self.sec();
                }

                // SED
                0xF8 => {
                    self.sed();
                }

                // SEI
                0x78 => {
                    self.sei();
                }

                // STA
                0x85 | 0x95 | 0x8D | 0x9D | 0x99 | 0x81 | 0x91 => {
                    self.sta(am);
                }

                // STX
                0x86 | 0x96 | 0x8E  => {
                    self.stx(am);
                }

                // STY
                0x84 | 0x94 | 0x8C => {
                    self.sty(am);
                }

                // TAX
                0xAA => {
                    self.tax();
                }

                // TAY
                0xA8 => {
                    self.tay();
                }

                // TSX
                0xBA => {
                    self.tsx();
                }

                // TXA
                0x8A => {
                    self.txa();
                } 

                // TXS
                0x9A => {
                    self.txs();
                }

                // TYA
                0x98 => {
                    self.tya();
                }

                // should not happen
                _ => todo!(),
            }

            // if the PC was not touched
            if self.program_counter == pc {
                self.program_counter += op.length;
            }
        }
    }

    /*
       Operations
    */

    fn adc(&mut self, mode: &AddressingMode) {
        let m = self.register_acc;
        let n = self.get_operand(mode);

        let (nc, mut carry) = if self.is_carry_set() {n.overflowing_add(1)} else {(n, false)};
       
        (self.register_acc, carry) = if !carry {m.overflowing_add(nc)} else {(m, true)};

        self.update_carry(carry);
        self.update_zero_and_negative_flags(self.register_acc);
        self.update_overflow( (m^self.register_acc)&(n^self.register_acc)&0x80 == 0x80);
    }

    fn and(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_acc = self.register_acc & value;
        self.update_zero_and_negative_flags(self.register_acc);
    }

    fn asl(&mut self, mode: &AddressingMode) {
        let mut value = self.get_operand(mode);
        self.update_carry(value & 0b1000_0000 == 0b1000_0000);
        value = value << 1;
        self.store_operand(mode, value);
        self.update_zero_and_negative_flags(value);
    }

    fn bcc(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        if !self.is_carry_set() {
            self.program_counter += value as u16;
        }
    }

    fn bcs(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        if self.is_carry_set() {
            self.program_counter += value as u16;
        }
    }

    fn beq(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        if self.is_zero_set() {
            self.program_counter += value as u16;
        }
    }

    fn bmi(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        if self.is_negative_set() {
            self.program_counter += value as u16;
        }
    }

    fn bne(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        if !self.is_zero_set() {
            self.program_counter += value as u16;
        }
    }

    fn bpl(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        if !self.is_negative_set() {
            self.program_counter += value as u16;
        }
    }

    fn bvc(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        if !self.is_overflow_set() {
            self.program_counter += value as u16;
        }
    }

    fn bvs(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        if self.is_overflow_set() {
            self.program_counter += value as u16;
        }
    }

    fn bit(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        
        self.register_status = self.register_status & 0b0011_1110 
            | value & 0b1100_0000 // copy bit 6 and 7 to the status
            | if value & self.register_acc == 0 {0b0000_0010} else {0}; // set zero if zero
    }

    fn clc(&mut self) {
        self.update_carry(false);
    }

    fn cld(&mut self) {
        self.update_decimal(false);
    }

    fn cli(&mut self) {
        self.update_interrupt_disable(false);
    }

    fn clv(&mut self) {
        self.update_overflow(false);
    }

    fn cmp(&mut self, mode: &AddressingMode) {
        let mut value = self.get_operand(mode);
        let overflow;
        (value, overflow) = self.register_acc.overflowing_sub(value);
        self.update_carry(!overflow);
        self.update_zero_and_negative_flags(value);
    }

    fn cpx(&mut self, mode: &AddressingMode) {
        let mut value = self.get_operand(mode);
        let overflow;
        (value, overflow) = self.register_x.overflowing_sub(value);
        self.update_carry(!overflow);
        self.update_zero_and_negative_flags(value);
    }

    fn cpy(&mut self, mode: &AddressingMode) {
        let mut value = self.get_operand(mode);
        let overflow;
        (value, overflow) = self.register_y.overflowing_sub(value);
        self.update_carry(!overflow);
        self.update_zero_and_negative_flags(value);
    }
    
    fn dec(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let (value, _) = self.mem_read(addr).overflowing_sub(1);
        self.mem_write(addr, value);
        self.update_zero_and_negative_flags(value);
    }

    fn dex(&mut self) {
        (self.register_x, _) = self.register_x.overflowing_sub(1);
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn dey(&mut self) {
        (self.register_y, _) = self.register_y.overflowing_sub(1);
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn eor(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        self.register_acc ^= value;
        self.update_zero_and_negative_flags(self.register_acc);
    }

    fn inc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let (value, _) = self.mem_read(addr).overflowing_add(1);
        self.mem_write(addr, value);
        self.update_zero_and_negative_flags(value);
    }

    fn inx(&mut self) {
        (self.register_x, _) = self.register_x.overflowing_add(1);
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn iny(&mut self) {
        (self.register_y, _) = self.register_y.overflowing_add(1);
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn jmp(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.program_counter = addr;
    }

    fn jsr(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.push_u16(self.program_counter + 3); // TODO remove hardcoded
        self.program_counter = addr;
    }

    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_acc = value;
        self.update_zero_and_negative_flags(value);
    }

    fn ldx(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);

        self.register_x = value;
        self.update_zero_and_negative_flags(value);
    }

    fn ldy(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);

        self.register_y = value;
        self.update_zero_and_negative_flags(value);
    }

    fn lsr(&mut self, mode: &AddressingMode) {
        let mut value = self.get_operand(mode);
        self.update_carry(value & 0b1 == 0b1);
        value = value >> 1;
        self.store_operand(mode, value);
        self.update_zero_and_negative_flags(value);
    }

    fn nop(&mut self) {
    }

    fn ora(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        self.register_acc |= value;
        self.update_zero_and_negative_flags(self.register_acc);
    }

    fn pha(&mut self) {
        self.push(self.register_acc);
    }

    fn php(&mut self) {
        self.push(self.register_status);
    }

    fn pla(&mut self) {
        self.register_acc = self.pull();
        self.update_zero_and_negative_flags(self.register_acc);
    }

    fn plp(&mut self) {
        self.register_status = self.pull();
    }
    
    fn rol(&mut self, mode: &AddressingMode) {
        let carry = self.is_carry_set();
        let mut value = self.get_operand(mode);
        self.update_carry(value & 0b1000_0000 == 0b1000_0000);
        value = (value << 1) | (if carry {1} else {0});
        self.store_operand(mode, value);
        self.update_zero_and_negative_flags(value);
    }

    fn ror(&mut self, mode: &AddressingMode) {
        let carry = self.is_carry_set();
        let mut value = self.get_operand(mode);
        self.update_carry(value & 0b1 == 0b1);
        value = (value >> 1) | (if carry {0b1000_0000} else {0});
        self.store_operand(mode, value);
        self.update_zero_and_negative_flags(value);
    }

    fn rts(&mut self) {
        let addr = self.pull_u16();
        self.program_counter = addr;
    }

    fn sbc(&mut self, mode: &AddressingMode) {
        let m = self.register_acc;
        let n = !self.get_operand(mode); // invert to substract

        let (nc, mut carry) = if self.is_carry_set() {n.overflowing_add(1)} else {(n, false)};
       
        (self.register_acc, carry) = if !carry {m.overflowing_add(nc)} else {(m, true)};

        self.update_carry(carry);
        self.update_zero_and_negative_flags(self.register_acc);
        self.update_overflow( (m^self.register_acc)&(n^self.register_acc)&0x80 == 0x80);
    }

    fn sec(&mut self) {
        self.update_carry(true);
    }

    fn sed(&mut self) {
        self.update_decimal(true);
    }

    fn sei(&mut self) {
        self.update_interrupt_disable(true);
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_acc);
    }

    fn stx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_x);
    }

    fn sty(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_y);
    }

    fn tax(&mut self) {
        self.register_x = self.register_acc;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn tay(&mut self) {
        self.register_y = self.register_acc;
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn tsx(&mut self) {
        self.register_x = self.stack_pointer;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn txa(&mut self) {
        self.register_acc = self.register_x;
        self.update_zero_and_negative_flags(self.register_acc);
    }
    
    fn txs(&mut self) {
        self.stack_pointer = self.register_x;
    }

    fn tya(&mut self) {
        self.register_acc = self.register_y;
        self.update_zero_and_negative_flags(self.register_acc);
    }

    
    /*
        Misc
    */
    fn update_carry(&mut self, val: bool) {
        if val {
            self.register_status |= 0b0000_0001;
        } else {
            self.register_status &= 0b1111_1110;
        }
    }

    fn update_decimal(&mut self, val: bool) {
        if val {
            self.register_status |= 0b0000_1000;
        } else {
            self.register_status &= 0b1111_0111;
        }
    }

    fn update_interrupt_disable(&mut self, val: bool) {
        if val {
            self.register_status |= 0b0000_0100;
        } else {
            self.register_status &= 0b1111_1011;
        }
    }

    fn update_overflow(&mut self, val: bool) {
        if val {
            self.register_status |= 0b0100_0000;
        } else {
            self.register_status &= 0b1011_1111;
        }
    }

    fn update_zero_and_negative_flags(&mut self, result: u8) {
        if result == 0 {
            self.register_status |= 0b0000_0010;
        } else {
            self.register_status &= 0b1111_1101;
        }

        if result & 0b1000_0000 != 0 {
            self.register_status |= 0b1000_0000;
        } else {
            self.register_status &= 0b0111_1111;
        }
    }

    fn is_carry_set(&self) -> bool {
        self.register_status & 0b1 == 0b1
    }

    fn is_zero_set(&self) -> bool {
        self.register_status & 0b10 == 0b10
    }

    fn is_negative_set(&self) -> bool {
        self.register_status & 0b1000_0000 == 0b1000_0000
    }

    fn is_overflow_set(&self) -> bool {
        self.register_status & 0b0100_0000 == 0b0100_0000
    }

    fn is_decimal_set(&self) -> bool {
        self.register_status & 0b0000_1000 == 0b0000_1000
    }

    fn is_interrupt_disable_set(&self) -> bool {
        self.register_status & 0b0000_0100 == 0b0000_0100
    }


    /*
    7  bit  0
    ---- ----
    NVss DIZC
    |||| ||||
    |||| |||+- Carry
    |||| ||+-- Zero
    |||| |+--- Interrupt Disable
    |||| +---- Decimal
    ||++------ No CPU effect, see: the B flag
    |+-------- Overflow
    +--------- Negative
    */
}





#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.register_acc, 0x05);
        assert!(cpu.register_status & 0b0000_0010 == 0b00);
        assert!(cpu.register_status & 0b1000_0000 == 0);
    }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.register_status & 0b0000_0010 == 0b10);
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu: Cpu6502 = Cpu6502::new();
        cpu.load(vec![0xaa, 0x00]);
        cpu.reset();
        cpu.register_acc = 10;
        cpu.run();

        assert_eq!(cpu.register_x, 10)
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 0xc1)
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = Cpu6502::new();
        cpu.load(vec![0xe8, 0xe8, 0x00]);
        cpu.reset();
        cpu.register_x = 0xff;
        cpu.run();

        assert_eq!(cpu.register_x, 1)
    }

    #[test]
    fn test_lda_from_memory() {
        let mut cpu = Cpu6502::new();
        cpu.mem_write(0x10, 0x55);

        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);

        assert_eq!(cpu.register_acc, 0x55);
    }

    #[test]
    fn test_adc_immediate_carry() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0x01, 0x69, 0xff, 0x00]);
        assert_eq!(cpu.register_acc, 0x00);
        assert!(cpu.is_carry_set());
    }

    #[test]
    fn test_adc_immediate_no_carry() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0x01, 0x69, 0xfe, 0x00]);
        assert_eq!(cpu.register_acc, 0xff);
        assert!(!cpu.is_carry_set());
    }

    #[test]
    fn test_adc_immediate_plus_carry() {
        let mut cpu = Cpu6502::new();
        cpu.load(vec![0xa9, 0x01, 0x69, 0xfd, 0x00]);
        cpu.reset();
        cpu.update_carry(true);
        cpu.run();
        assert_eq!(cpu.register_acc, 0xff);
        assert!(!cpu.is_carry_set());
    }

    #[test]
    fn test_adc_lowlevel() {
        fn eval_adc(cpu: &mut Cpu6502, m: u8, n: u8, res:u8, carry: bool, overflow: bool) {
            cpu.load_and_run(vec![0xa9, m, 0x69, n, 0x00]);
            assert!(cpu.is_carry_set() == carry);
            assert!(cpu.is_overflow_set() == overflow);
            assert_eq!(cpu.register_acc, res);
        }
        
        let mut cpu = Cpu6502::new();
        eval_adc(&mut cpu, 80, 16, 96, false, false);
        eval_adc(&mut cpu, 80, 80, 160, false, true);
        eval_adc(&mut cpu, 80, 144, 224, false, false);
        eval_adc(&mut cpu, 80, 208, (288-256) as u8, true, false);
        eval_adc(&mut cpu, 208, 16, 224, false, false);
        eval_adc(&mut cpu, 208, 80, (288-256) as u8, true, false);
        eval_adc(&mut cpu, 208, 144, (352-256) as u8, true, true);
        eval_adc(&mut cpu, 208, 208, (416-256) as u8, true, false);
    }

    #[test]
    fn test_and_immediate() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0b00011111, 0x29, 0b11111000, 0x00]);
        assert_eq!(cpu.register_acc, 0b00011000);
        assert!(!cpu.is_zero_set());
    }

    #[test]
    fn test_asl_acc_no_carry() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0b00011111, 0x0A, 0x00]);
        assert_eq!(cpu.register_acc, 0b00111110);
        assert!(!cpu.is_carry_set());
        assert!(!cpu.is_zero_set());
    }

    #[test]
    fn test_asl_acc_carry() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0b10011111, 0x0A, 0x00]);
        assert_eq!(cpu.register_acc, 0b00111110);
        assert!(cpu.is_carry_set());
        assert!(!cpu.is_zero_set());
    }

    #[test]
    fn test_bcc() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0b00011111, 0x0A, 0x90, 0x03, 0x00, 0xE8, 0x00]);
        assert_eq!(cpu.register_x, 1)
    }

    #[test]
    fn test_bcs() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0b10011111, 0x0A, 0xB0, 0x03, 0x00, 0xE8, 0x00]);
        assert_eq!(cpu.register_x, 1)
    }

    #[test]
    fn test_beq() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0, 0x0A, 0xF0, 0x03, 0x00, 0xE8, 0x00]);
        assert_eq!(cpu.register_x, 1)
    }

    #[test]
    fn test_bmi() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0b0100_0000, 0x0A, 0x30, 0x03, 0x00, 0xE8, 0x00]);
        assert_eq!(cpu.register_x, 1)
    }

    #[test]
    fn test_bne() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 1, 0x0A, 0xD0, 0x03, 0x00, 0xE8, 0x00]);
        assert_eq!(cpu.register_x, 1)
    }

    #[test]
    fn test_bpl() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0b0010_0000, 0x0A, 0x10, 0x03, 0x00, 0xE8, 0x00]);
        assert_eq!(cpu.register_x, 1)
    }

    /* 
    TODO implement when overflow is set somehow

    #[test]
    fn test_bvc() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0b0010_0000, 0x0A, 0x50, 0x03, 0x00, 0xE8, 0x00]);
        assert_eq!(cpu.register_x, 1)
    }

    #[test]
    fn test_bvs() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0b0010_0000, 0x0A, 0x70, 0x03, 0x00, 0xE8, 0x00]);
        assert_eq!(cpu.register_x, 1)
    }
    */

    #[test]
    fn test_bit() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b10011111, 0x85, 0x01, 0xA9, 0b11000000, 0x24, 0x01, 0x00]);
        
        assert!(!cpu.is_zero_set());
        assert!(cpu.is_negative_set());
        assert!(!cpu.is_overflow_set());
    }

    #[test]
    fn test_bit_zero() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b01011111, 0x85, 0x01, 0xA9, 0b00000000, 0x24, 0x01, 0x00]);
        
        assert!(cpu.is_zero_set());
        assert!(!cpu.is_negative_set());
        assert!(cpu.is_overflow_set());
    }

    #[test]
    fn test_clc() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0b10011111, 0x0A, 0x18, 0x00]);
        assert!(!cpu.is_carry_set());
    }

    /* 
    TODO implement when interrupt and decimal bits are set
    #[test]
    fn test_cld() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b11011111, 0x85, 0x01, 0xA9, 0b11000000, 0x24, 0x01, 0xB8, 0x00]);
        
        assert!(!cpu.is_overflow_set());
    }

    #[test]
    fn test_cli() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b11011111, 0x85, 0x01, 0xA9, 0b11000000, 0x24, 0x01, 0xB8, 0x00]);
        
        assert!(!cpu.is_overflow_set());
    }
    */

    #[test]
    fn test_clv() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b11011111, 0x85, 0x01, 0xA9, 0b11000000, 0x24, 0x01, 0xB8, 0x00]);
        
        assert!(!cpu.is_overflow_set());
    }

    #[test]
    fn test_cmp_eq() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b10011111, 0xC9, 0b10011111, 0x00]);
        assert!(cpu.is_carry_set());
        assert!(cpu.is_zero_set());
        assert!(!cpu.is_negative_set());
    }

    #[test]
    fn test_cmp_ne() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b10011111, 0xC9, 0b00011111, 0x00]);
        assert!(cpu.is_carry_set());
        assert!(!cpu.is_zero_set());
        assert!(cpu.is_negative_set());
    }

    #[test]
    fn test_cmp_ne2() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b00011111, 0xC9, 0b10011111, 0x00]);
        assert!(!cpu.is_carry_set());
        assert!(!cpu.is_zero_set());
        assert!(cpu.is_negative_set());
    }

    #[test]
    fn test_cpx_eq() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b10011111, 0xAA, 0xE0, 0b10011111, 0x00]);
        assert!(cpu.is_carry_set());
        assert!(cpu.is_zero_set());
        assert!(!cpu.is_negative_set());
    }

    #[test]
    fn test_cpx_ne() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b10011111, 0xAA, 0xE0, 0b00011111, 0x00]);
        assert!(cpu.is_carry_set());
        assert!(!cpu.is_zero_set());
        assert!(cpu.is_negative_set());
    }

    #[test]
    fn test_cpx_ne2() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b00011111, 0xAA, 0xE0, 0b10011111, 0x00]);
        assert!(!cpu.is_carry_set());
        assert!(!cpu.is_zero_set());
        assert!(cpu.is_negative_set());
    }

    /* 
    TODO when tay is ready
    fn test_cpy_eq() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b10011111, 0xA8, 0xC0, 0b10011111, 0x00]);
        assert!(cpu.is_carry_set());
        assert!(cpu.is_zero_set());
        assert!(!cpu.is_negative_set());
    }

    #[test]
    fn test_cpy_ne() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b10011111, 0xA8, 0xC0, 0b00011111, 0x00]);
        assert!(cpu.is_carry_set());
        assert!(!cpu.is_zero_set());
        assert!(cpu.is_negative_set());
    }

    #[test]
    fn test_cpy_ne2() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b00011111, 0xA8, 0xC0, 0b10011111, 0x00]);
        assert!(!cpu.is_carry_set());
        assert!(!cpu.is_zero_set());
        assert!(cpu.is_negative_set());
    }
    */

    #[test]
    fn test_dec() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0x02, 0x85, 0x01, 0xC6, 0x01, 0x00]);
        //                           | LDA         STA         DEC         
        assert_eq!(cpu.mem_read(0x01), 1);
    }

    #[test]
    fn test_dex() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0x02, 0xAA, 0xCA, 0x00]);
        //                           | LDA         TAX   DEX         
        assert_eq!(cpu.register_x, 1);
    }

    #[test]
    fn test_dey() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0x88, 0x88, 0x00]);
        //                           | DEY   DEY         
        assert_eq!(cpu.register_y, 254);
    }

    #[test]
    fn test_eor() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b01010101, 0x49, 0b11111111, 0x00]);
        //                           | LDA               EOR            
        assert_eq!(cpu.register_acc, 0b10101010);
        assert!(!cpu.is_zero_set());
        assert!(cpu.is_negative_set());
    }

    #[test]
    fn test_inc() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0x02, 0x85, 0x01, 0xE6, 0x01, 0x00]);
        //                           | LDA         STA         INC         
        assert_eq!(cpu.mem_read(0x01), 3);
    }

    #[test]
    fn test_inx() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0x02, 0xAA, 0xE8, 0x00]);
        //                           | LDA         TAX   INX         
        assert_eq!(cpu.register_x, 3);
    }

    #[test]
    fn test_iny() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xC8, 0xC8, 0x00]);
        //                           | INY   INY         
        assert_eq!(cpu.register_y, 2);
    }

    #[test]
    fn test_jmp() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0x4C, 0x04, 0x80, 0x00, 0xE8, 0x00]);
        //                           | JMP               trap  INX
        assert_eq!(cpu.register_x, 1);
    }

    #[test]
    fn test_jmp_indirect() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0x6C, 0x04, 0x80, 0x00, 0x07, 0x80, 0x00, 0xE8, 0x00]);
        //                           | JMP               trap  addr        trap  INX      
        assert_eq!(cpu.register_x, 1);
    }

    #[test]
    fn test_jsr() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0x20, 0x04, 0x80, 0x00, 0xE8, 0x00]);
        //                           | JSR               trap  INX
        assert_eq!(cpu.register_x, 1);
        assert_eq!(cpu.stack_pointer, 253);
        assert_eq!(cpu.mem_read_u16(0x01FE), 0x8003);
    }

    #[test]
    fn test_ldx() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA2, 0b10000001, 0x00]);
        //                           | LDX              

        assert_eq!(cpu.register_x, 0b10000001);
        assert!(!cpu.is_zero_set());
        assert!(cpu.is_negative_set());
    }

    #[test]
    fn test_ldy() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA0, 0, 0x00]);
        //                           | LDY             

        assert_eq!(cpu.register_x, 0);
        assert!(cpu.is_zero_set());
        assert!(!cpu.is_negative_set());
    }

    #[test]
    fn test_lsr_acc_no_carry() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b00011110, 0x4A, 0x00]);
         //                          | LDA               LSR
        assert_eq!(cpu.register_acc, 0b00001111); 
        assert!(!cpu.is_carry_set());
        assert!(!cpu.is_zero_set());
    }

    #[test]
    fn test_lsr_acc_carry() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b00011111, 0x4A, 0x00]);
         //                          | LDA               LSR
        assert_eq!(cpu.register_acc, 0b00001111);
        assert!(cpu.is_carry_set());
        assert!(!cpu.is_zero_set());
    }

    #[test]
    fn test_ora() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b01010101, 0x09, 0b10101010, 0x00]);
        //                           | LDA               ORA            
        assert_eq!(cpu.register_acc, 0b11111111);
        assert!(!cpu.is_zero_set());
        assert!(cpu.is_negative_set());
    }

    #[test]
    fn test_pha() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b01010101, 0x48, 0x00]);
        //                           | LDA               PHA            
        assert_eq!(cpu.stack_pointer, 254);
        assert_eq!(cpu.mem_read(0x01FF), 0b01010101);
    }

    #[test]
    fn test_php() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b10011111, 0xC9, 0b00011111, 0x08, 0x00]);
        //                           | LDA               CMP               PHP          
        assert_eq!(cpu.stack_pointer, 254);
        assert_eq!(cpu.mem_read(0x01FF), 0b10000001); // carry and negative
    }

    #[test]
    fn test_pla() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 232, 0x48, 0xA9, 1, 0x68, 0x00]);
        //                           | LDA        PHA   LDA      PLA     
        assert_eq!(cpu.register_acc, 232);
        assert!(!cpu.is_zero_set());
        assert!(cpu.is_negative_set());
    }

    #[test]
    fn test_plp() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 232, 0x48, 0xA9, 1, 0x28, 0x00]);
        //                           | LDA        PHA   LDA      PLP     
        assert_eq!(cpu.register_status, 232);
    }

    #[test]
    fn test_rol() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b1011_1110, 0x2A, 0x00]);
        //                           | LDA                ROL 
        assert_eq!(cpu.register_acc, 0b0111_1100);
        assert!(cpu.is_carry_set());
    }
    
    #[test]
    fn test_ror() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 0b1011_1111, 0x6A, 0x00]);
        //                           | LDA                ROR 
        assert_eq!(cpu.register_acc, 0b0101_1111);
        assert!(cpu.is_carry_set());
    }

    #[test]
    fn test_rts() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0x20, 0x05, 0x80, 0xE8, 0x00, 0xE8, 0x60, 0x00]);
        //                           | JSR               INX   trap  INX   RTS
        assert_eq!(cpu.register_x, 2);
        assert_eq!(cpu.stack_pointer, 255);
    }

    #[test]
    fn test_sbc_immediate_carry() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 0x02, 0xE9, 0x01, 0x00]);
        //                           | LDA         SBC
        assert_eq!(cpu.register_acc, 0x00); // SBC executed with borrow=!carry
        assert!(cpu.is_carry_set());
    }

    #[test]
    fn test_sbc_immediate_no_carry() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xa9, 56, 0xE9, 21, 0x00]);
         //                           | LDA      SBC
        assert_eq!(cpu.register_acc, 35-1); // SBC executed with borrow=!carry
        assert!(cpu.is_carry_set());
    }
    
    #[test]
    fn test_sbc_lowlevel() {
        fn eval_sbc(cpu: &mut Cpu6502, m: u8, n: u8, res:u8, carry: bool, overflow: bool) {
            cpu.load_and_run(vec![0xA9, 0b1, 0x4A, 0xA9, m, 0xE9, n, 0x00]);
            //                           | LDA        LSR   LDA      SBC 
            assert!(cpu.is_carry_set() == carry);
            assert!(cpu.is_overflow_set() == overflow);
            assert_eq!(cpu.register_acc, res);
        }
        
        let mut cpu = Cpu6502::new();
        eval_sbc(&mut cpu, 80, 240, 96, false, false);
        eval_sbc(&mut cpu, 80, 176, 160, false, true);
        eval_sbc(&mut cpu, 80, 112, 224, false, false);
        eval_sbc(&mut cpu, 80, 48, 32, true, false);
        eval_sbc(&mut cpu, 208, 240, 224, false, false);
        eval_sbc(&mut cpu, 208, 176, 32, true, false);
        eval_sbc(&mut cpu, 208, 112, 96, true, true);
        eval_sbc(&mut cpu, 208, 48, 160, true, false);
    }

    #[test]
    fn test_sec() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0x38, 0x00]);
        assert!(cpu.is_carry_set());
    }

    #[test]
    fn test_sed() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xF8, 0x00]);
        assert!(cpu.is_decimal_set());
    }

    #[test]
    fn test_sei() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0x78, 0x00]);
        assert!(cpu.is_interrupt_disable_set());
    }

    #[test]
    fn test_sta() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 123, 0x85, 0x01, 0x00]);
        //                           | LDA         STA  
        assert_eq!(cpu.mem_read(0x01), 123);
    }

    #[test]
    fn test_stx() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA2, 123, 0x86, 0x01, 0x00]);
        //                           | LDX         STX  
        assert_eq!(cpu.mem_read(0x01), 123);
    }

    #[test]
    fn test_sty() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA0, 123, 0x84, 0x01, 0x00]);
        //                           | LDY         STY  
        assert_eq!(cpu.mem_read(0x01), 123);
    }

    #[test]
    fn test_tax() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 123, 0xAA, 0x00]);
        //                           | LDA         TAX         
        assert_eq!(cpu.register_x, 123);
    }

    #[test]
    fn test_tay() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 123, 0xA8, 0x00]);
        //                           | LDA        TAY         
        assert_eq!(cpu.register_y, 123);
    }

    #[test]
    fn test_tsx() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0x48, 0xBA, 0x00]);
        //                           | PHA   TSX         
        assert_eq!(cpu.register_x, 254);
    }

    #[test]
    fn test_txa() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 123, 0xAA, 0xA9, 12, 0x8A, 0x00]);
        //                           | LDA         TAX  LDA       TXA
        assert_eq!(cpu.register_acc, 123);
    }

    #[test]
    fn test_txs() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 123, 0xAA, 0xA9, 12, 0x9A, 0x00]);
        //                           | LDA         TAX  LDA       TXS
        assert_eq!(cpu.stack_pointer, 123);
    }

    #[test]
    fn test_tya() {
        let mut cpu = Cpu6502::new();
        cpu.load_and_run(vec![0xA9, 123, 0xA8, 0xA9, 12, 0x98, 0x00]);
        //                           | LDA        TAY   LDA       TYA      
        assert_eq!(cpu.register_acc, 123);
    }

}
