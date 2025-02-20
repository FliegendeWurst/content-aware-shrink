use std::env;

static SLOW: &'static str = "--slow";
static USAGE: &'static str =
	"USAGE: content-aware-shrink [--slow] <file> [<cuts>|MAX [<save-interval>|@<target-height>|@<target-width>x<target-height>]]";

use content_aware_shrink::*;
use image::ImageReader;

fn main() {
	let mut args = env::args_os().skip(1).collect::<Vec<_>>();
	if args.len() < 1 {
		println!("{USAGE}");
		std::process::exit(1);
	}
	let _ = env_logger::builder()
		.parse_filters("content_aware_shrink=info")
		.try_init();
	let slow_mode = args.iter().any(|x| x == SLOW);
	let path = args.iter().filter(|&x| x != SLOW).next().cloned();
	if let Some(p) = &path {
		args.remove_item(p);
	}
	if path.is_none() {
		println!("{USAGE}");
		std::process::exit(1);
	}
	let mut img = ImageReader::open(path.unwrap()).unwrap().decode().unwrap().into_rgb8();
	let cuts = args
		.iter()
		.filter(|&x| x != SLOW)
		.next()
		.map(|x| match x.to_string_lossy().as_ref() {
			"MAX" => u32::MAX,
			x => x.parse().expect("invalid number of cuts"),
		})
		.unwrap_or(u32::MAX);
	let (save_interval, save_height, save_width) = args
		.iter()
		.filter(|&x| x != SLOW)
		.skip(1)
		.next()
		.map(|x| match x.to_string_lossy().as_ref() {
			x if x.starts_with('@') && x.contains('x') => {
				let parts = x[1..].split_once('x').unwrap();
				(
					0,
					parts.1.parse().expect("invalid target height"),
					parts.0.parse().expect("invalid target width"),
				)
			},
			x if x.starts_with('@') => (0, x[1..].parse().expect("invalid target height"), 0),
			x => (x.parse().expect("invalid number of cuts"), 0, 0),
		})
		.unwrap_or((20, 0, 0));
	let cuts = (img.height() - 2).min(cuts);

	let mask = None; //ImageReader::open("/home/arne/Downloads/mask.png").unwrap().decode().unwrap().into_rgb8();

	if slow_mode {
		for i in 0..cuts {
			let new_img = cut_once(&img, Axis::Horizontal);
			println!("iteration {i}, img {} x {}", new_img.width(), new_img.height());
			if (save_interval != 0 && i % save_interval == 0) || new_img.height() == save_height {
				new_img.save(format!("iteration_s{}.png", i)).unwrap();
				if new_img.height() == save_height && new_img.width() == save_width {
					break;
				}
			}
			img = new_img;
		}
	} else {
		let mut img_data = ImageData::new(img, mask);
		img_data.cut_to_dimensions(save_width as _, save_height as _);
		let img = img_data.get_img();
		img.save("result.png").unwrap();
	}
}
