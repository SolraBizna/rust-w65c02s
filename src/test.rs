use std::collections::VecDeque;
use super::*;

// the other other other S3!
struct SimpleSerialSystem {
    serial_in: VecDeque<u8>,
    serial_out: Vec<u8>,
    ram: [u8; 65536],
}

impl SimpleSerialSystem {
    pub fn new(serial_in: Vec<u8>,
               ram_image: &[(u16, Vec<u8>)]) -> SimpleSerialSystem {
        let serial_in = serial_in.into();
        let mut ram = [0; 65536];
        for (base, bytes) in ram_image.iter() {
            let mut addr = *base as usize;
            for byte in bytes.iter() {
                ram[addr] = *byte;
                addr = addr + 1;
            }
        }
        SimpleSerialSystem {
            serial_in, serial_out: Vec::new(), ram: ram
        }
    }
}

impl System for SimpleSerialSystem {
    fn read(&mut self, cpu: &mut W65C02S, addr: u16) -> u8 {
        if addr == 0 {
            match self.serial_in.pop_front() {
                Some(x) => x,
                None => {
                    cpu.set_p(cpu.get_p() | P_V);
                    0xFF
                }
            }
        }
        else {
            self.ram[addr as usize]
        }
    }
    fn write(&mut self, _: &mut W65C02S, addr: u16, value: u8) {
        if addr == 0 { self.serial_out.push(value) }
        else { self.ram[addr as usize] = value }
    }
}

fn run_test(ram_image: &[(u16, Vec<u8>)], serial_in: Vec<u8>,
            expected_serial_out: &[u8]) {
    let mut system = SimpleSerialSystem::new(serial_in, ram_image);
    let mut cpu = W65C02S::new();
    let mut stopped = false;
    for _ in 0..1000000 {
        if cpu.step(&mut system) == State::Stopped {
            stopped = true;
            break
        }
    }
    if !stopped {
        panic!("Test ran for one million cycles.")
    }
    assert_eq!(system.serial_out, expected_serial_out);
}

#[test]
fn instant_stp() {
    run_test(&[
        (RESET_VECTOR, vec![0x00, 0x02]),
        (0x0200, vec![
            op::STP
        ]),
    ], [].to_vec(), &[]);
}

#[test]
fn dead_simple_hello_world() {
    run_test(&[
        (RESET_VECTOR, vec![0x00, 0x02]),
        (0x0200, vec![
            op::LDA_IMM, b'H',
            op::STA_ZP, 0x00,
            op::LDA_IMM, b'i',
            op::STA_ZP, 0x00,
            op::STP
        ]),
    ], [].to_vec(), b"Hi");
}

#[test]
fn less_simple_hello_world() {
    run_test(&[
        (RESET_VECTOR, vec![0x00, 0x02]),
        (0x0001, b"Hello World!\n".to_vec()),
        (0x0200, vec![
            op::LDX_IMM, 0,
            // (branch leads here)
            op::LDA_ZPX, 0x01,
            op::BNE, 1, // skip the STP if the byte is nonzero
            op::STP,
            op::STA_ZP, 0x00,
            op::INC_X,
            op::BRA, -10 as i8 as u8,
        ]),
    ], [].to_vec(), b"Hello World!\n");
}

#[test]
fn pass_through_hello_world() {
    run_test(&[
        (RESET_VECTOR, vec![0x00, 0x02]),
        (0x0200, vec![
            // (branch leads here)
            op::LDA_ZP, 0x00,
            op::BVC, 1, // skip the STP if the byte is nonzero
            op::STP,
            op::STA_ZP, 0x00,
            op::JMP_ABS, 0x00, 0x02,
        ]),
    ], b"Hello World!\n".to_vec(), b"Hello World!\n");
}
