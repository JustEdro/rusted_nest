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

    pub fn reset(&mut self) {
        self.register_acc = 0;
        self.register_x = 0;
        self.register_status = 0;

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

    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        let operand_pc = self.program_counter + 1;
        match mode {
            AddressingMode::Immediate | AddressingMode::Relative => operand_pc,

            AddressingMode::ZeroPage => self.mem_read(operand_pc) as u16,

            AddressingMode::Absolute => self.mem_read_u16(operand_pc),

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

            AddressingMode::NoneAddressing => {
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

            // pick a function
            // TODO there should be a better way of doing 
            match opscode {
                // LDA
                0xA9 | 0xA5 | 0xB5 | 0xAD | 0xBD | 0xB9 | 0xA1 | 0xB1 => {
                    self.lda(am);
                }
                
                // STA
                0x85 | 0x95  => {
                    self.sta(am);
                }

                // TAX
                0xAA => self.tax(),

                // INX
                0xE8 => self.inx(),

                // BRK
                0x00 => {
                    return;
                }
                _ => todo!(),
            }

            self.program_counter += op.length;
        }
    }

    /*
       Operations
    */

    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_acc = value;
        self.update_zero_and_negative_flags(self.register_acc);
    }

    fn tax(&mut self) {
        self.register_x = self.register_acc;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn inx(&mut self) {
        (self.register_x, _) = self.register_x.overflowing_add(1);
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_acc);
    }

    /*
        Misc
    */

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
}
