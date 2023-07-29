use std::{fs::File, io::BufReader};

use image::ImageFormat;
use rand::{rngs::SmallRng, SeedableRng};

use content_aware_shrink::*;

#[cfg(test)]
fn init() {
	let _ = env_logger::builder().is_test(true).try_init();
}

#[test]
fn noise_test() {
	init();
	let mut img = image::load(BufReader::new(File::open("tests/random.png").unwrap()), ImageFormat::Png)
		.unwrap()
		.into_rgb8();

	let mut img_data = ImageData::new(&img, Axis::Horizontal, SmallRng::from_seed([0x57; 32]));

    for n in 1..99 {
		img = cut_once(&img, Axis::Horizontal);
		img_data.cut_once();
		assert_eq!(img, img_data.get_img(), "nth iteration wrong (n = {n})");
	}
	assert_eq!(2, img_data.get_img().height());
    assert_eq!(2, img.height());
	// cannot shrink further
}

#[test]
#[ignore] // this test is for experiments only
fn picture_test() {
	let mut img = image::load(
		BufReader::new(File::open("/home/arne/Downloads/9vn44xj74eu91.jpg").unwrap()),
		ImageFormat::Jpeg,
	)
	.unwrap()
	.into_rgb8();

	let mut img_data = ImageData::new(&img, Axis::Horizontal, SmallRng::from_seed([0x17; 32]));
	for n in 1..99 {
		img = cut_once(&img, Axis::Horizontal);
		img_data.cut_once();
		if img != img_data.get_img() {
			panic!("nth iteration wrong (n = {n})");
		}
	}
	assert_eq!(2, img_data.get_img().height());
	// cannot shrink further
}
