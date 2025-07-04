#![no_std]

pub mod mpu6050_lib{
	pub mod lib;
	pub mod mpu;
	pub mod bits;
}

pub mod common_protocol{
    pub mod can_transfer;
}