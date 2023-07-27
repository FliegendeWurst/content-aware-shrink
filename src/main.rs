use std::{
	collections::{BinaryHeap, HashSet},
	fs::{self, File},
	io::BufReader,
	thread,
};

use arrayvec::ArrayVec;
use image::{GenericImage, GenericImageView, ImageBuffer, ImageFormat, Rgba};
use pathfinding::prelude::dijkstra;

type Image = ImageBuffer<Rgba<u8>, Vec<u8>>;

#[test]
fn noise_test() {
	let img = image::load(BufReader::new(File::open("random.png").unwrap()), ImageFormat::Png)
		.unwrap()
		.into_rgba8();

	let img1 = cut_once(&img, Axis::Horizontal);
	let mut img_data = ImageData::new(&img, Axis::Horizontal);
	img_data.cut_once();
	assert_eq!(img1, img_data.get_img(), "first iteration wrong");

	let img2 = cut_once(&img1, Axis::Horizontal);
	img_data.cut_once();
	assert_eq!(img2, img_data.get_img(), "second iteration wrong");

	let img3 = cut_once(&img2, Axis::Horizontal);
	img_data.cut_once();
	img3.save("/tmp/a.png");
	img_data.save_img("/tmp/b.png");
	// assert_eq!(img3, img_data.get_img(), "third iteration wrong");
	if img3 != img_data.get_img() {
		panic!("third iteration wrong");
	}

	for _ in 4..99 {
		img_data.cut_once();
	}
	assert_eq!(2, img_data.get_img().height());
	// cannot shrink further
}

fn main() {
	let path = "/tmp/a.png";
	let img = image::load(BufReader::new(File::open(path).unwrap()), ImageFormat::Jpeg)
		.unwrap()
		.into_rgba8();
	let mut img_data = ImageData::new(&img, Axis::Horizontal);
	let cuts = img.height() - 2;
	let mut threads = vec![];
	for i in 0..cuts {
		img_data.cut_once();
		if i % 8 == 1 && i == 183843828 {
			let img = img_data.get_img();
			println!("Saving iteration {}", i);
			threads.push(thread::spawn(move || {
				img.save(&format!("x_iteration{}.png", i)).unwrap();
			}));
		}
	}
	threads.into_iter().for_each(|x| x.join().unwrap());
	/*
	for i in 0..cuts {
		let new_img = cut_once(&img, Axis::Horizontal);
		println!("current height {}", new_img.height());
		/*
		if new_img.height() == 1080 {
			new_img.save("result.png").unwrap();
			return;
		}
		*/
		break;
		if i % 5 == 0 {
			new_img.save(format!("iteration{}.png", i)).unwrap();
		}
		img = new_img;
	}
	*/
}

struct ImageData {
	img_height: usize,
	img_width: usize,
	nodes: Vec<PixelNode>,
	valid_nodes: usize,
	rescan: HashSet<usize>,
	d: Vec<u32>,
	p: Vec<usize>,
}

impl ImageData {
	fn new(img: &Image, axis: Axis) -> Self {
		assert_eq!(Axis::Horizontal, axis);
		let mut nodes = Vec::with_capacity(img.height() as usize * img.width() as usize);
		let end_node = (img.height() * img.width() + 1) as usize;

		for y in 0..img.height() {
			for x in 0..img.width() {
				let mut neighbors = ArrayVec::new();
				if x < img.width() - 1 {
					if y > 0 {
						let idx = (y - 1) * img.width() + x + 1;
						neighbors.push(idx as usize);
					}
					let idx = y * img.width() + x + 1;
					neighbors.push(idx as usize);
					if y < img.height() - 1 {
						let idx = (y + 1) * img.width() + x + 1;
						neighbors.push(idx as usize);
					}
				} else {
					neighbors.push(end_node);
				}

				let cost = if y == 0 || y == img.height() - 1 {
					u32::MAX
				} else {
					calc_cost(img.get_pixel(x, y - 1), img.get_pixel(x, y + 1))
				};

				let mut neighbors_back = ArrayVec::new();
				if x > 0 {
					if y > 0 {
						let idx = (y - 1) * img.width() + x - 1;
						neighbors_back.push(idx as usize);
					}
					let idx = y * img.width() + x - 1;
					neighbors_back.push(idx as usize);
					if y < img.height() - 1 {
						let idx = (y + 1) * img.width() + x - 1;
						neighbors_back.push(idx as usize);
					}
				}

				nodes.push(PixelNode {
					color: *img.get_pixel(x, y),
					neighbors,
					neighbors_back,
					cost,
				});
			}
		}

		let start_node = PixelNode {
			color: *img.get_pixel(0, 0),
			neighbors_back: ArrayVec::new(),
			neighbors: ArrayVec::new(),
			cost: 0,
		};
		nodes.push(start_node);

		let end_node = PixelNode {
			color: *img.get_pixel(0, 0),
			neighbors: ArrayVec::new(),
			neighbors_back: ArrayVec::new(),
			cost: 0,
		};
		nodes.push(end_node);

		let d = vec![u32::MAX; nodes.len()];
		let p = vec![0; nodes.len()];

		ImageData {
			nodes,
			d,
			p,
			img_height: img.height() as usize,
			img_width: img.width() as usize,
			valid_nodes: (img.height() * img.width()) as usize,
			rescan: HashSet::new(),
		}
	}

	fn cut_once(&mut self) {
		let d = &mut self.d;
		let p = &mut self.p;

		let mut q = BinaryHeap::new();
		//q.push((u32::MAX, self.nodes.len() - 2));
		d[self.nodes.len() - 2] = 0;

		for y in 1..self.img_height - 1 {
			let idx = y * self.img_width;
			if !self.nodes[idx].neighbors.is_empty() {
				q.push((u32::MAX - self.nodes[idx].cost, idx));
				p[idx] = self.nodes.len() - 2;
				d[idx] = self.nodes[idx].cost;
			}
		}
		for x in self.rescan.drain() {
			if self.nodes[x].neighbors.is_empty() {
				continue;
			}
			// println!("rescan {} {}", x, d[x]);
			q.push((u32::MAX - d[x], x));
		}

		let mut best_end = u32::MAX;
		while let Some((total_cost, src)) = q.pop() {
			//assert!(total_cost != u32::MAX || src == self.nodes.len() - 2);
			if u32::MAX - total_cost >= best_end {
				// println!("total cost = {}", u32::MAX - total_cost);
				break;
			}
			for &next_node in &self.nodes[src].neighbors {
				let cost = self.nodes[next_node].cost;
				if d[src].saturating_add(cost) < d[next_node] {
					d[next_node] = d[src] + cost;
					p[next_node] = src;
					q.push((u32::MAX - d[next_node], next_node));
					if next_node == self.nodes.len() - 1 {
						best_end = d[next_node];
						// println!("NEW ALGO: {best_end}");
					}
				}
			}
		}

		// reconstruct cut starting at end
		let mut cut = vec![];
		let mut node = self.nodes.len() - 1;
		while p[node] != 0 {
			//println!("processing {node} {}", p[node]);
			cut.push((
				p[node],
				self.nodes[p[p[node]]].neighbors.iter().position(|&x| x == p[node]),
			));
			node = p[node];
		}
		cut.pop();

		let mut tainted = HashSet::<usize>::new();
		let mut q = HashSet::new();

		d[self.nodes.len() - 1] = u32::MAX;
		// println!("new cut {cut:?}");
		let mut last_idx = None;
		for (idx, choice) in cut.into_iter().rev() {
			// println!("y = {}, ", idx / self.img_width);
			// println!("cut on {} {}", idx % self.img_width, idx / self.img_width);
			d[idx] = u32::MAX;
			q.insert(idx);
			for &n in &self.nodes[idx].neighbors {
				d[n] = u32::MAX;
			}
			q.extend(&self.nodes[idx].neighbors);

			for next in self.nodes[idx].neighbors.clone() {
				assert!(self.nodes[next].neighbors_back.remove_item(&idx).is_some() || next == self.nodes.len() - 1);
			}

			self.nodes[idx].neighbors.clear();
			self.valid_nodes -= 1;
			// restore correct weights for neighboring nodes:
			// find correct pixel above
			let mut above = idx;
			while above >= self.img_width {
				above -= self.img_width;
				if !self.nodes[above].neighbors.is_empty() {
					break;
				}
			}
			let mut above2 = above;
			while above2 >= self.img_width {
				above2 -= self.img_width;
				if !self.nodes[above2].neighbors.is_empty() {
					break;
				}
			}
			// find correct pixel below
			let mut below = idx;
			while below < self.nodes.len() - self.img_width - 2 {
				below += self.img_width;
				if !self.nodes[below].neighbors.is_empty() {
					break;
				}
			}
			let mut below2 = below;
			while below2 < self.nodes.len() - self.img_width - 2 {
				below2 += self.img_width;
				if !self.nodes[below2].neighbors.is_empty() {
					break;
				}
			}
			// println!("found {} {} {} {}", above2, above, below, below2);

			// there are two valid pixels above
			if above != idx {
				q.insert(above);
			}
			if above2 != above {
				self.nodes[above].cost = calc_cost(&self.nodes[above2].color, &self.nodes[below].color);
				d[above] = u32::MAX;
				q.insert(above2);
			}
			if below != idx {
				q.insert(below);
			}
			if below2 != below {
				self.nodes[below].cost = calc_cost(&self.nodes[above].color, &self.nodes[below2].color);
				d[below] = u32::MAX;
				q.insert(below2);
			}

			// update neighbors of preceding pixels
			for prev in self.nodes[idx].neighbors_back.clone() {
				//println!("want to get {idx} {} {:?}", prev, self.nodes[prev].neighbors);
				let i = self.nodes[prev].neighbors.iter().position(|&x| x == idx);
				if i.is_none() {
					continue;
				}
				let i = i.unwrap();
				self.nodes[prev].neighbors.remove_item(&idx);
				if prev == self.nodes.len() - 2 {
					continue;
				}
				// tainted.insert(prev);
				// tainted.extend(&self.nodes[prev].neighbors);
				match i {
					0 => {
						if above != idx {
							// println!("case 0, at idx {idx} and prev {prev} and above {above}");
							self.nodes[prev].neighbors.insert(0, above);
							//assert!(self.nodes[above].neighbors_back.remove_item(&idx).is_some());
							self.nodes[above].neighbors_back.push(prev);
							// assert_eq!(3, self.nodes[above].neighbors_back.len());
						}
					},
					1 if choice == Some(0) => { // last_idx == Some(idx - 1 + self.img_width) => {
						if below2 != below {
							// println!("hopefully 2 {choice:?}");
							// println!("case 1, at idx {idx} and prev {prev} and below2 {below2}, {:?}", self.nodes[below2].neighbors_back);
							self.nodes[prev].neighbors.push(below2);
							//assert!(self.nodes[below2].neighbors_back.remove_item(&idx).is_some());
							self.nodes[below2].neighbors_back.insert(0, prev);
							// assert_eq!(3, self.nodes[below2].neighbors_back.len());
						}
					},
					1 if choice == Some(2) => { // last_idx == Some(idx - 1 - self.img_width) => {
						if above2 != above {
							// println!("hopefully 0 {choice:?}");
							// println!("case 1b, at idx {idx} and prev {prev} and below2 {below2}, {:?}", self.nodes[below2].neighbors_back);

							self.nodes[prev].neighbors.insert(0, above2);
							//assert!(self.nodes[below2].neighbors_back.remove_item(&idx).is_some());
							self.nodes[above2].neighbors_back.push(prev);
							// assert_eq!(3, self.nodes[below2].neighbors_back.len());
						}
					},
					2 => {
						if below != idx {
							// println!("case 2, at idx {idx} and prev {prev} and below {below}, {:?}", self.nodes[below].neighbors_back);
							self.nodes[prev].neighbors.push(below);
							//assert!(self.nodes[below].neighbors_back.remove_item(&idx).is_some());
							self.nodes[below].neighbors_back.push(prev);
							// assert_eq!(3, self.nodes[below].neighbors_back.len());
						}
					},
					_ if prev < self.img_width || prev >= self.img_width * (self.img_height - 1) => {},
					_ => panic!("unexpected index in neighbors of {prev} cost {} {:?} {i} @ idx = {idx} last = {:?}, choice {choice:?} valid = {}", self.nodes[prev].cost, self.nodes[prev].neighbors, last_idx, self.valid_nodes)
				}
			}
			last_idx = Some(idx);
		}
		/*
		for i in 0..self.nodes.len() - 2 {
			d[i] = u32::MAX;
		}
		*/
		// println!();
		while !q.is_empty() {
			let mut new_q = HashSet::new();
			for x in q {
				d[x] = u32::MAX;
				tainted.insert(x);
				for &neigh in &self.nodes[x].neighbors {
					if self.nodes[p[neigh]].neighbors.is_empty() {
						continue;
					}
					d[neigh] = u32::MAX;
					if tainted.contains(&neigh) {
						continue;
					}
					tainted.insert(neigh);
					new_q.insert(neigh);
				}
			}
			q = new_q;
		}
		for x in tainted {
			for &prev in &self.nodes[x].neighbors_back {
				if d[prev] != u32::MAX {
					// println!("rescan! {} {}", prev, u32::MAX - d[prev]);
					self.rescan.insert(prev);
				}
			}
			//self.nodes[x].neighbors_back.clear();
		}
		// println!("new valid {}", self.valid_nodes);
	}

	fn get_img(&self) -> Image {
		let height = self.valid_nodes / self.img_width;
		let mut buffer = Image::new(self.img_width as u32, height as _);

		for x in 0..self.img_width {
			let mut y = 0;
			for y_check in 0..self.img_height {
				let node = &self.nodes[y_check * self.img_width + x];
				if node.neighbors.is_empty() {
					continue;
				}
				buffer.put_pixel(x as u32, y, node.color);
				y += 1;
			}
		}
		buffer
	}

	fn save_img(&self, path: &str) {
		self.get_img().save(path).unwrap();
	}
}

fn calc_cost(a: &Rgba<u8>, b: &Rgba<u8>) -> u32 {
	let cost: u32 = (0..3).map(|i| a.0[i].abs_diff(b.0[i]).pow(2) as u32).sum();
	(100.0 * (cost as f32).sqrt()) as u32
}

struct PixelNode {
	color: Rgba<u8>,
	cost: u32,
	neighbors: ArrayVec<usize, 4>,
	neighbors_back: ArrayVec<usize, 4>,
}

fn cut_many(img: &ImageBuffer<Rgba<u8>, Vec<u8>>, axis: Axis, n: usize) -> ImageBuffer<Rgba<u8>, Vec<u8>> {
	let mut img_data = ImageData::new(img, axis);

	todo!()
}

fn cut_once(img: &ImageBuffer<Rgba<u8>, Vec<u8>>, axis: Axis) -> ImageBuffer<Rgba<u8>, Vec<u8>> {
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
	println!("OLD ALGO: {}", cut_path.1);
	let pixels = cut_path.0;
	let mut exclude = match axis {
		Axis::Vertical => vec![0; img.height() as _],
		Axis::Horizontal => vec![0; img.width() as _],
	};
	for p in pixels {
		match p {
			Node::Pixel(x, y) => {
				//img.put_pixel(x, y, Rgba([0xff, 0, 0, 0xff]));
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
	//img.save("output.png").unwrap();
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
enum Axis {
	Vertical,
	Horizontal,
}

trait VecRemoveItem<T, U> {
	fn remove_item(&mut self, item: &U) -> Option<T>
	where
		T: PartialEq<U>;
}

impl<T: PartialEq<U>, U> VecRemoveItem<T, U> for Vec<T> {
	fn remove_item(&mut self, item: &U) -> Option<T> {
		self.iter().position(|n| n == item).map(|idx| self.remove(idx))
	}
}

impl<T: PartialEq<U>, U, const Z: usize> VecRemoveItem<T, U> for ArrayVec<T, Z> {
	fn remove_item(&mut self, item: &U) -> Option<T> {
		self.iter().position(|n| n == item).map(|idx| self.remove(idx))
	}
}
