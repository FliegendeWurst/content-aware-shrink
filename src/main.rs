use std::{env, thread};

use image::io::Reader as ImageReader;

static SLOW: &'static str = "--slow";
static USAGE: &'static str =
	"USAGE: content-aware-shrink [--slow] <file> [<cuts>|MAX [<save-interval>|@<target-height>]]";

use content_aware_shrink::*;

fn main() {
	let mut args = env::args_os().skip(1).collect::<Vec<_>>();
	if args.len() < 1 {
		println!("{USAGE}");
		std::process::exit(1);
	}
	let slow_mode = args.iter().any(|x| x == SLOW);
	let path = args.iter().filter(|&x| x != SLOW).next().cloned();
	if let Some(p) = &path {
		args.remove_item(p);
	}
	let cuts = args
		.iter()
		.filter(|&x| x != SLOW)
		.next()
		.map(|x| match x.to_string_lossy().as_ref() {
			"MAX" => u32::MAX,
			x => x.parse().expect("invalid number of cuts"),
		})
		.unwrap_or(u32::MAX);
	let (save_interval, save_height) = args
		.iter()
		.filter(|&x| x != SLOW)
		.skip(1)
		.next()
		.map(|x| match x.to_string_lossy().as_ref() {
			x if x.starts_with('@') => (0, x[1..].parse().expect("invalid target height")),
			x => (x.parse().expect("invalid number of cuts"), 0),
		})
		.unwrap_or((20, 0));
	if path.is_none() {
		println!("{USAGE}");
		std::process::exit(1);
	}
	let mut img = ImageReader::open(path.unwrap()).unwrap().decode().unwrap().into_rgb8();
	let cuts = (img.height() - 2).min(cuts);
	if slow_mode {
		for i in 0..cuts {
			let new_img = cut_once(&img, Axis::Horizontal);
			println!("iteration {i}, img height {}", new_img.height());
			if (save_interval != 0 && i % save_interval == 0) || new_img.height() == save_height {
				new_img.save(format!("iteration_s{}.png", i)).unwrap();
				if new_img.height() == save_height {
					break;
				}
			}
			img = new_img;
		}
	} else {
		let mut img_data = ImageData::new(&img, Axis::Horizontal, rand::thread_rng());
		let mut threads = vec![];
		for i in 0..cuts {
			img_data.cut_once();
			println!("iteration {i}, img height {}", img.height() - 1 - i);
			if (save_interval != 0 && i % save_interval == 0) || img.height() - 1 - i == save_height {
				let img = img_data.get_img();
				let exit = img.height() == save_height;
				threads.push(thread::spawn(move || {
					img.save(&format!("iteration_f{}.png", i)).unwrap();
				}));
				if exit {
					break;
				}
			}
		}
		threads.into_iter().for_each(|x| x.join().unwrap());
	}
}
