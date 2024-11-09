use std::error::Error;

use dary_heap::QuaternaryHeap;
use image::{ImageBuffer, Rgb};
use log::{debug, info};
use pathfinding::prelude::dijkstra;

pub type Image = ImageBuffer<Rgb<u8>, Vec<u8>>;
type Index = u32;

const MAX_COST: u32 = u32::MAX / 4;
const COST_PER_PIXEL: u32 = 32;

pub struct ImageData {
	pub img_height: usize,
	pub img_width: usize,
	i: Vec<Vec<(u32, u32, u32, u32)>>,
	mask: Option<Image>,
	img: Image,
}

impl ImageData {
	pub fn new(img: Image, mask_img: Option<Image>) -> Self {
		let mask = mask_img.as_ref();
		let mut i = vec![];
		for y in 0..img.height() {
			let i_line = (0..img.width())
				.map(|x| {
					let h_cost = if y == 0 || y == img.height() - 1 {
						MAX_COST
					} else if mask
						.map(|mask| *mask.get_pixel(x, y) == Rgb([0x00; 3]))
						.unwrap_or(false)
					{
						0
					} else {
						calc_cost(img.get_pixel(x, y - 1), img.get_pixel(x, y + 1))
					};
					let v_cost = if x == 0 || x == img.width() - 1 {
						MAX_COST
					} else if mask
						.map(|mask| *mask.get_pixel(x, y) == Rgb([0x00; 3]))
						.unwrap_or(false)
					{
						0
					} else {
						calc_cost(img.get_pixel(x - 1, y), img.get_pixel(x + 1, y))
					};
					(x, y, h_cost, v_cost)
				})
				.collect();
			i.push(i_line);
		}

		ImageData {
			img_height: img.height() as usize,
			img_width: img.width() as usize,
			i,
			mask: mask_img,
			img,
		}
	}

	pub fn cut_to_dimensions(&mut self, width: usize, height: usize) {
		let aspect_ratio = width as f32 / height as f32;
		loop {
			let current_width = self.i[0].len();
			let current_height = self.i.len();
			if current_width == width && current_height == height {
				return;
			}
			info!("cut to {current_width} x {current_height}");
			if current_width < width {
				panic!("request to shrink to width larger than current");
			}
			if current_height < height {
				panic!("request to shrink to height larger than current");
			}
			// try to get the best aspect ratio
			let aspect_ratio_h = (current_width as f32) / (current_height as f32 - 1.0);
			let aspect_ratio_v = (current_width as f32 - 1.0) / (current_height as f32);
			let h_permitted = current_height != height;
			let v_permitted = current_width != width;
			let axis = if h_permitted && (aspect_ratio_h - aspect_ratio).abs() < (aspect_ratio_v - aspect_ratio).abs() {
				Axis::Horizontal
			} else if v_permitted && (aspect_ratio_v - aspect_ratio).abs() < (aspect_ratio_h - aspect_ratio).abs() {
				Axis::Vertical
			} else if h_permitted {
				Axis::Horizontal
			} else if v_permitted {
				Axis::Vertical
			} else {
				unreachable!()
			};
			self.cut_once(axis);
		}
	}

	pub fn cut_once(&mut self, axis: Axis) {
		// binary: 34.442s (std) 34.474s
		// ternary: 34.610s
		// quaternary: 33.797 / 34.433 / (34.128 with_capacity)
		// quinary: 39.910s
		// octonary: 40.245s

		let mut q = QuaternaryHeap::with_capacity(16384); // BinaryHeap::new();

		let total_pixels = self.i.iter().map(|x| x.len()).sum::<usize>();
		let start_node = total_pixels;
		let end_node = total_pixels + 1;

		let current_width = self.i[0].len();
		let current_height = self.i.len();

		assert_eq!(total_pixels, current_height * current_width);

		let mut d = vec![u32::MAX; total_pixels + 2];
		let mut p = vec![0; total_pixels + 2];

		d[start_node] = 0;

		macro_rules! init_layer {
			($var:ident, $range:expr, $idx:expr, $cost_idx:tt) => {
				for $var in $range {
					let idx = $idx;
					let y = idx / current_width;
					let x = idx % current_width;
					if self.i[y][x].$cost_idx != MAX_COST {
						q.push((u32::MAX - self.i[y][x].$cost_idx, idx as Index));
						p[idx] = start_node;
						d[idx] = self.i[y][x].$cost_idx;
					}
				}
			};
		}
		match axis {
			Axis::Vertical => init_layer!(x, 1..current_width - 1, x, 3),
			Axis::Horizontal => init_layer!(y, 1..current_height - 1, y * current_width, 2),
		}

		let mut best_end = u32::MAX;
		while let Some((total_cost, src)) = q.pop() {
			let src = src as usize;
			if u32::MAX - total_cost >= best_end {
				break;
			}
			let (x, y) = (src % current_width, src / current_width);

			// check whether we are done
			if match axis {
				Axis::Vertical => y == current_height - 1,
				Axis::Horizontal => x == current_width - 1,
			} {
				if d[src] < best_end {
					best_end = d[src];
					p[end_node] = src;
					continue;
				}
			}

			// go to next nodes
			macro_rules! check_node {
				($x:expr, $y:expr, $cost_idx:tt) => {
					let next_node = $y * current_width + $x;
					let cost = self.i[$y][$x].$cost_idx;
					if cost == MAX_COST {
						continue;
					}
					if d[src] + cost < d[next_node] {
						d[next_node] = d[src] + cost;
						p[next_node] = src;
						q.push((u32::MAX - d[next_node], next_node as Index));
					}
				};
			}
			match axis {
				Axis::Vertical => {
					if x > 0 {
						check_node!(x - 1, y + 1, 3);
					}
					check_node!(x, y + 1, 3);
					if x < current_width - 1 {
						check_node!(x + 1, y + 1, 3);
					}
				},
				Axis::Horizontal => {
					if y > 0 {
						check_node!(x + 1, y - 1, 2);
					}
					check_node!(x + 1, y, 2);
					if y < current_height - 1 {
						check_node!(x + 1, y + 1, 2);
					}
				},
			}
		}

		assert!(best_end < u32::MAX);

		// reconstruct cut starting at end
		let mut cut = vec![];
		let mut node = end_node;
		while p[node] != start_node {
			cut.push(p[node]);
			node = p[node];
		}

		// fixup i matrix
		for idx in cut {
			let (mut x, mut y) = (idx % current_width, idx / current_width);
			let (old_x, old_y) = (x, y);
			match axis {
				Axis::Vertical => {
					while x < current_width - 1 {
						self.i[y][x] = self.i[y][x + 1];
						x += 1;
					}
				},
				Axis::Horizontal => {
					while y < current_height - 1 {
						self.i[y][x] = self.i[y + 1][x];
						y += 1;
					}
				},
			}
			x = old_x;
			y = old_y;
			macro_rules! recalculate {
				($x:expr, $y:expr) => {{
					// recalculate cost for replaced pixel
					let h_cost = if y == 0 || y == current_height - 1 {
						MAX_COST
					} else if self
						.mask
						.as_ref()
						.map(|mask| *mask.get_pixel(x as _, y as _) == Rgb([0x00; 3]))
						.unwrap_or(false)
					{
						0
					} else {
						let (x2, y2, _, _) = self.i[old_y - 1][old_x];
						let (x3, y3, _, _) = self.i[old_y + 1][old_x];
						calc_cost(self.img.get_pixel(x2, y2), self.img.get_pixel(x3, y3))
					};
					let v_cost = if x == 0 || x == current_width - 1 {
						MAX_COST
					} else if self
						.mask
						.as_ref()
						.map(|mask| *mask.get_pixel(x as _, y as _) == Rgb([0x00; 3]))
						.unwrap_or(false)
					{
						0
					} else {
						let (x2, y2, _, _) = self.i[old_y][old_x - 1];
						let (x3, y3, _, _) = self.i[old_y][old_x + 1];
						calc_cost(self.img.get_pixel(x2, y2), self.img.get_pixel(x3, y3))
					};
					self.i[y][x].2 = h_cost;
					self.i[y][x].3 = v_cost;
				}};
			}
			recalculate!(x, y);
			match axis {
				Axis::Vertical => recalculate!(x - 1, y),
				Axis::Horizontal => recalculate!(x, y - 1),
			}
		}
		// shrink i matrix
		match axis {
			Axis::Vertical => {
				self.i.iter_mut().for_each(|x| {
					x.pop();
				});
			},
			Axis::Horizontal => {
				self.i.pop();
			},
		}
	}

	pub fn get_img(&self) -> Image {
		let height = self.i.len();
		let width = self.i[0].len();
		let mut buffer = Image::new(width as _, height as _);

		for y in 0..height {
			for x in 0..width {
				let (orig_x, orig_y, _, _) = self.i[y][x];
				buffer.put_pixel(x as _, y as _, *self.img.get_pixel(orig_x, orig_y));
			}
		}
		buffer
	}

	pub fn save_img(&self, path: &str) -> Result<(), Box<dyn Error>> {
		self.get_img().save(path)?;
		Ok(())
	}
}

fn calc_cost(a: &Rgb<u8>, b: &Rgb<u8>) -> u32 {
	let cost: u32 = (0..3).map(|i| (a.0[i].abs_diff(b.0[i]) as u32).pow(2)).sum();
	COST_PER_PIXEL + (100.0 * (cost as f32).sqrt()) as u32
}

pub fn cut_once(img: &Image, axis: Axis) -> Image {
	let cut_path = dijkstra(
		&Node::Start,
		|node| match (*node, axis) {
			(Node::Start, Axis::Vertical) => (1..img.width() - 1)
				.map(|x| {
					let pixel_a = img.get_pixel(x - 1, 0);
					let pixel_b = img.get_pixel(x + 1, 0);
					let cost = calc_cost(pixel_a, pixel_b);
					(Node::Pixel(x, 0), cost)
				})
				.collect::<Vec<_>>(),
			(Node::Start, Axis::Horizontal) => (1..img.height() - 1)
				.map(|y| {
					let pixel_a = img.get_pixel(0, y - 1);
					let pixel_b = img.get_pixel(0, y + 1);
					let cost = calc_cost(pixel_a, pixel_b);
					(Node::Pixel(0, y), cost)
				})
				.collect::<Vec<_>>(),
			(Node::Pixel(x, y), Axis::Vertical) => {
				if y == img.height() - 1 {
					return vec![(Node::Finish, 0)];
				}
				let mut next = vec![];
				for dx in [-1, 0, 1] {
					if let Some(x) = x.checked_add_signed(dx) {
						if x > 0 && x < img.width() - 1 {
							let pixel_a = img.get_pixel(x - 1, y + 1);
							let pixel_b = img.get_pixel(x + 1, y + 1);
							let cost = calc_cost(pixel_a, pixel_b);
							next.push((Node::Pixel(x, y + 1), cost));
						}
					}
				}
				next
			},
			(Node::Pixel(x, y), Axis::Horizontal) => {
				if x == img.width() - 1 {
					return vec![(Node::Finish, 0)];
				}
				let mut next = vec![];
				for dy in [-1, 0, 1] {
					if let Some(y) = y.checked_add_signed(dy) {
						if y > 0 && y < img.height() - 1 {
							let pixel_a = img.get_pixel(x + 1, y - 1);
							let pixel_b = img.get_pixel(x + 1, y + 1);
							let cost = calc_cost(pixel_a, pixel_b);
							next.push((Node::Pixel(x + 1, y), cost));
						}
					}
				}
				next
			},
			(Node::Finish, _) => vec![],
		},
		|x| *x == Node::Finish,
	);
	let cut_path = cut_path.unwrap();
	debug!("OLD ALGO: {}", cut_path.1);
	let pixels = cut_path.0;
	let mut exclude = match axis {
		Axis::Vertical => vec![0; img.height() as _],
		Axis::Horizontal => vec![0; img.width() as _],
	};
	for p in pixels {
		match p {
			Node::Pixel(x, y) => {
				//img.put_pixel(x, y, Rgb([0xff, 0, 0, 0xff]));
				match axis {
					Axis::Vertical => exclude[y as usize] = x,
					Axis::Horizontal => exclude[x as usize] = y,
				}
				// print!("{}, ", y);
			},
			_ => {},
		}
	}
	// println!();
	match axis {
		Axis::Vertical => {
			let mut new_img = ImageBuffer::new(img.width() - 1, img.height());
			for y in 0..img.height() {
				let mut new_x = 0;
				for x in 0..img.width() {
					if exclude[y as usize] == x {
						continue;
					}
					new_img.put_pixel(new_x, y, *img.get_pixel(x, y));
					new_x += 1;
				}
			}
			new_img
		},
		Axis::Horizontal => {
			let mut new_img = ImageBuffer::new(img.width(), img.height() - 1);
			for y in 0..img.height() {
				for x in 0..img.width() {
					if exclude[x as usize] == y {
						continue;
					}
					let real_y = if exclude[x as usize] < y { y - 1 } else { y };
					new_img.put_pixel(x, real_y, *img.get_pixel(x, y));
				}
			}
			new_img
		},
	}
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Node {
	Start,
	Pixel(u32, u32),
	Finish,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(unused)]
pub enum Axis {
	Vertical,
	Horizontal,
}

pub trait VecRemoveItem<T, U> {
	fn remove_item(&mut self, item: &U) -> Option<T>
	where
		T: PartialEq<U>;
}

impl<T: PartialEq<U>, U> VecRemoveItem<T, U> for Vec<T> {
	fn remove_item(&mut self, item: &U) -> Option<T> {
		self.iter().position(|n| n == item).map(|idx| self.remove(idx))
	}
}

impl VecRemoveItem<Index, Index> for [Index; 3] {
	fn remove_item(&mut self, item: &Index) -> Option<Index> {
		let item = *item;
		if self[2] == item {
			self[2] = Index::MAX;
			Some(item)
		} else if self[1] == item {
			self[1] = Index::MAX;
			self.sort();
			Some(item)
		} else if self[0] == item {
			self[0] = Index::MAX;
			self.sort();
			Some(item)
		} else {
			None
		}
	}
}
