use std::collections::{BinaryHeap, HashSet, VecDeque};

use arrayvec::ArrayVec;

use image::{ImageBuffer, Rgb};
use log::{debug, trace};
use pathfinding::prelude::dijkstra;
use rand::Rng;

pub type Image = ImageBuffer<Rgb<u8>, Vec<u8>>;
type Index = u32;

pub struct ImageData<R: Rng> {
	pub img_height: usize,
	pub img_width: usize,
	nodes: Vec<PixelNode>,
	valid_nodes: usize,
	rescan: HashSet<usize>,
	d: Vec<u32>,
	p: Vec<usize>,
	rng: R,
}

impl<R: Rng> ImageData<R> {
	pub fn new(img: &Image, axis: Axis, rng: R) -> Self {
		assert_eq!(Axis::Horizontal, axis);
		let mut nodes = Vec::with_capacity(img.height() as usize * img.width() as usize);
		let end_node = (img.height() * img.width() + 1) as Index;

		for y in 0..img.height() {
			for x in 0..img.width() {
				let mut neighbors = ArrayVec::new();
				if x < img.width() - 1 {
					if y > 0 {
						let idx = (y - 1) * img.width() + x + 1;
						neighbors.push(idx as Index);
					}
					let idx = y * img.width() + x + 1;
					neighbors.push(idx as Index);
					if y < img.height() - 1 {
						let idx = (y + 1) * img.width() + x + 1;
						neighbors.push(idx as Index);
					}
				} else {
					neighbors.push(end_node as Index);
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
						neighbors_back.push(idx as Index);
					}
					let idx = y * img.width() + x - 1;
					neighbors_back.push(idx as Index);
					if y < img.height() - 1 {
						let idx = (y + 1) * img.width() + x - 1;
						neighbors_back.push(idx as Index);
					}
				}

				nodes.push(PixelNode {
					color: *img.get_pixel(x, y),
					neighbors,
					neighbors_back,
					cost,
					tainted: false,
				});
			}
		}

		let start_node = PixelNode {
			color: *img.get_pixel(0, 0),
			neighbors_back: ArrayVec::new(),
			neighbors: ArrayVec::new(),
			cost: 0,
			tainted: false,
		};
		nodes.push(start_node);

		let end_node = PixelNode {
			color: *img.get_pixel(0, 0),
			neighbors: ArrayVec::new(),
			neighbors_back: ArrayVec::new(),
			cost: 0,
			tainted: false,
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
			rng,
		}
	}

	pub fn cut_once(&mut self) {
		let d = &mut self.d;
		let p = &mut self.p;
		let rng = &mut self.rng;

		let mut q = BinaryHeap::new();
		d[self.nodes.len() - 2] = 0;

		for y in 1..self.img_height - 1 {
			let idx = y * self.img_width;
			if !self.nodes[idx].neighbors.is_empty() {
				// println!("inserting {}", u32::MAX - self.nodes[idx].cost);
				q.add((
					u32::MAX - self.nodes[idx].cost,
					rng.gen_range(u32::MAX / 2..u32::MAX),
					idx as Index,
				));
				p[idx] = self.nodes.len() - 2;
				d[idx] = self.nodes[idx].cost;
			}
		}

		for x in self.rescan.drain() {
			if self.nodes[x].neighbors.is_empty() {
				continue;
			}
			q.add((u32::MAX - d[x], rng.gen_range(u32::MAX / 2..u32::MAX), x as Index));
		}

		let mut best_end = u32::MAX;
		while let Some((total_cost, _rand, src)) = q.remove_elem() {
			let src = src as usize;
			if src == usize::MAX {
				continue;
			}
			if u32::MAX - total_cost >= best_end {
				break;
			}
			for &next_node in &self.nodes[src].neighbors {
				let next_node = next_node as usize;
				let cost = self.nodes[next_node].cost;
				assert!(d[src] < u32::MAX);
				if cost == u32::MAX {
					continue;
				}
				if d[src] + cost < d[next_node] {
					d[next_node] = d[src] + cost;
					p[next_node] = src;
					q.add((u32::MAX - d[next_node], rng.gen(), next_node as Index));
					if next_node == self.nodes.len() - 1 {
						best_end = d[next_node];
						debug!("NEW ALGO: {best_end}");
					}
				}
			}
		}

		// reconstruct cut starting at end
		let mut cut = vec![];
		let mut node = self.nodes.len() - 1;
		while p[node] != 0 {
			cut.push((
				p[node],
				self.nodes[p[p[node]]]
					.neighbors
					.iter()
					.position(|&x| x == p[node] as Index),
			));
			node = p[node];
		}
		cut.pop();

		let mut q = Vec::new();

		d[self.nodes.len() - 1] = u32::MAX;
		let mut last_idx = None;
		for (idx, choice) in cut.into_iter().rev() {
			let idx = idx as usize;
			let y = idx / self.img_width;
			assert!(y > 0 && y < self.img_height - 1);
			// println!("cut on {} {}", idx % self.img_width, idx / self.img_width);
			d[idx] = u32::MAX;
			q.push(idx as Index);
			for &n in &self.nodes[idx].neighbors {
				d[n as usize] = u32::MAX;
			}
			q.extend(&self.nodes[idx].neighbors);

			let mut prev = idx as Index;
			for i in 0..self.nodes[idx].neighbors.len() {
				let next = self.nodes[idx].neighbors[i];
				if next == prev {
					continue;
				}
				prev = next;
				assert!(
					self.nodes[next as usize]
						.neighbors_back
						.remove_item(&(idx as Index))
						.is_some() || next == self.nodes.len() as Index - 1,
					"next = {next}, idx = {idx}, neighbors_back = {:?}",
					self.nodes[next as usize].neighbors_back
				);
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

			// there are two valid pixels above
			if above != idx {
				q.push(above as Index);
			}
			if above2 != above {
				self.nodes[above].cost = calc_cost(&self.nodes[above2].color, &self.nodes[below].color);
				d[above] = u32::MAX;
				q.push(above2 as Index);
			}
			if below != idx {
				q.push(below as Index);
			}
			if below2 != below {
				self.nodes[below].cost = calc_cost(&self.nodes[above].color, &self.nodes[below2].color);
				d[below] = u32::MAX;
				q.push(below2 as Index);
			}

			// update neighbors of preceding pixels
			for (prev_i, prev) in self.nodes[idx].neighbors_back.clone().into_iter().enumerate() {
				let prev = prev as usize;
				trace!(
					"want to get rid of {idx} {choice:?} {:?}",
					self.nodes[idx].neighbors_back
				);

				let i = self.nodes[prev].neighbors.iter().position(|&x| x == idx as u32);
				if i.is_none() {
					continue;
				}
				let i = i.unwrap();
				self.nodes[prev].neighbors.remove_item(&(idx as u32));
				if prev == self.nodes.len() - 2 {
					continue;
				}
				// p = prev
				// I = idx
				// X = p[I]
				match i {
					// want to get rid of 155 Some(1) [54, 254]

					// . .
					// . I
					// p .
					// . .
					0 => {
						if above != idx as usize {
							trace!("case 0, at idx {idx} and prev {prev} and above {above}");
							self.nodes[prev].neighbors.insert(0, above as Index);
							self.nodes[prev].neighbors.sort();
							//assert!(self.nodes[above].neighbors_back.remove_item(&idx).is_some());
							self.nodes[above].neighbors_back.remove_item(&(p[idx] as Index));
							self.nodes[above].neighbors_back.push(prev as u32);
							self.nodes[above].neighbors_back.sort();
							// assert_eq!(3, self.nodes[above].neighbors_back.len());
						}
					},
					// . .
					// . .
					// p I
					// X .
					// . .
					1 if choice == Some(0) && prev_i != 0 => {
						if below2 != below {
							trace!("case 1, at idx {idx} and prev {prev} and below2 {below2}, {:?}", self.nodes[below2].neighbors_back);
							self.nodes[prev].neighbors.push(below2 as Index);
							self.nodes[prev].neighbors.sort();
							self.nodes[below2].neighbors_back.remove_item(&(p[idx] as Index));
							self.nodes[below2].neighbors_back.insert(0, prev  as Index);
							self.nodes[below2].neighbors_back.sort();
						}
					},
					// p .
					// x I
					// x .
					1 if choice != Some(2) => {
						if below != idx {
							trace!("case 1a, at idx {idx} and prev {prev} {prev_i} {:?} and below {below}, {:?}", self.nodes[prev].neighbors, self.nodes[below].neighbors_back);
							self.nodes[prev].neighbors.push(below as Index);
							self.nodes[prev].neighbors.sort();
							self.nodes[below].neighbors_back.remove_item(&(p[idx] as Index));
							self.nodes[below].neighbors_back.insert(0, prev as Index);
							self.nodes[below].neighbors_back.sort();
						}
						assert!(prev < self.img_width);
					},
					// . .
					// X .
					// p I
					// . .
					1 if choice == Some(2) => {
						if above2 != above {
							trace!("case 1b, at idx {idx} and prev {prev} and above2 {above2}, {:?}", self.nodes[above2].neighbors_back);

							self.nodes[prev].neighbors.insert(0, above2 as Index);
							self.nodes[prev].neighbors.sort();
							trace!("result {:?}", self.nodes[prev].neighbors);
							//assert!(self.nodes[below2].neighbors_back.remove_item(&idx).is_some());
							self.nodes[above2].neighbors_back.remove_item(&(p[idx] as Index));
							self.nodes[above2].neighbors_back.push(prev as Index);
							self.nodes[above2].neighbors_back.sort();
							// assert_eq!(3, self.nodes[below2].neighbors_back.len());
						}
					},
					// . .
					// p .
					// . I
					// . .
					// . .
					2 => {
						if below != idx {
							trace!("case 2, at idx {idx} and prev {prev} and below {below}, {:?}", self.nodes[below].neighbors_back);
							self.nodes[prev].neighbors.push(below as Index);
							self.nodes[prev].neighbors.sort();
							self.nodes[below].neighbors_back.remove_item(&(p[idx] as Index));
							self.nodes[below].neighbors_back.push(prev as Index);
							self.nodes[below].neighbors_back.sort();
							trace!("resulting below {:?}", self.nodes[below].neighbors_back);
						}
					},
					_ if prev < self.img_width || prev >= self.img_width * (self.img_height - 1) => {},
					_ => panic!("unexpected index in neighbors of {prev} cost {} {:?} {i} @ idx = {idx} last = {:?}, choice {choice:?} valid = {}", self.nodes[prev].cost, self.nodes[prev].neighbors, last_idx, self.valid_nodes)
				}
			}
			last_idx = Some(idx);
		}
		// println!();
		while !q.is_empty() {
			let mut new_q = Vec::new();
			for x in q {
				let x = x as usize;
				d[x] = u32::MAX;
				self.nodes[x].tainted = true;
				for i in 0..self.nodes[x].neighbors.len() {
					let neigh = self.nodes[x].neighbors[i] as usize;
					if self.nodes[p[neigh]].neighbors.is_empty() {
						continue;
					}
					d[neigh] = u32::MAX;
					if self.nodes[neigh].tainted {
						continue;
					}
					self.nodes[neigh].tainted = true;
					new_q.push(neigh as Index);
				}
			}
			q = new_q;
		}
		for i in 0..self.nodes.len() {
			if !self.nodes[i].tainted {
				continue;
			}
			self.nodes[i].tainted = false;
			for &prev in &*self.nodes[i].neighbors_back {
				let prev = prev as usize;
				if d[prev] != u32::MAX {
					// println!("rescan! {} {}", prev, u32::MAX - d[prev]);
					self.rescan.insert(prev);
				}
			}
			//self.nodes[x].neighbors_back.clear();
		}
		// println!("new valid {}", self.valid_nodes);
	}

	pub fn get_img(&self) -> Image {
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

	pub fn save_img(&self, path: &str) {
		self.get_img().save(path).unwrap();
	}
}

fn calc_cost(a: &Rgb<u8>, b: &Rgb<u8>) -> u32 {
	let cost: u32 = (0..3).map(|i| (a.0[i].abs_diff(b.0[i]) as u32).pow(2)).sum();
	(100.0 * (cost as f32).sqrt()) as u32
}

struct PixelNode {
	// total size: 40 bytes (should be optimized further)
	color: Rgb<u8>,
	cost: u32,
	neighbors: ArrayVec<Index, 3>,
	neighbors_back: ArrayVec<Index, 3>,
	tainted: bool,
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

impl<T: PartialEq<U>, U, const Z: usize> VecRemoveItem<T, U> for ArrayVec<T, Z> {
	fn remove_item(&mut self, item: &U) -> Option<T> {
		self.iter().position(|n| n == item).map(|idx| self.remove(idx))
	}
}

type QueueItem = (u32, u32, u32);

trait NodeQueue<T> {
	fn remove_elem(&mut self) -> Option<T>;
	fn peek_elem(&mut self) -> Option<T>;

	fn add(&mut self, item: T);
}

impl NodeQueue<QueueItem> for BinaryHeap<QueueItem> {
	fn remove_elem(&mut self) -> Option<QueueItem> {
		self.pop()
	}

	fn peek_elem(&mut self) -> Option<QueueItem> {
		self.peek().copied()
	}

	fn add(&mut self, item: QueueItem) {
		self.push(item)
	}
}

impl<T> NodeQueue<T> for VecDeque<T> {
	fn remove_elem(&mut self) -> Option<T> {
		self.pop_front()
	}

	fn peek_elem(&mut self) -> Option<T> {
		todo!()
	}

	fn add(&mut self, item: T) {
		self.push_back(item)
	}
}
