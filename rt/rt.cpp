/**
	The program below generates 512 spheres of different radiuses and renders
	and image of this set of spheres using ray tracing for orthographic camera:
	It generates a single ray for each image plane pixel and finds closest 
	intersection of this ray with a set of spheres. If the intersection is found 
	the program sets pixel color to the color of the sphere. If there are no 
	intersections the color is set to RGB=(0.1 0.1 0.1)
	
	On 2.4GHz processor the program takes ~15-16s to generate and image.
	The task is to optimize the program as much as possible. Make sure it generates 
	the same output.
*/

#include <iostream>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <vector>
#include <stack>

#include "oiio/include/OpenImageIO/imageio.h"
#include "rt_structs.h"
#include "VolumeTree.h"

// Output image dimensions
std::uint32_t const kImageWidth = 2048;
std::uint32_t const kImageHeight = 2048;
// Number of spheres to render
std::uint32_t const kNumSpheres = 512;

#define RAND_FLOAT (((float)std::rand()) / RAND_MAX)
#define LEFT -10.f
#define BOTTOM -10.f
#define WIDTH 20.f
#define HEIGHT 20.f
#define NEAR -10.f
#define FAR 10.f

// Randomly generate sphere array of num_spheres items
void generate_spheres(sphere* spheres, std::uint32_t num_spheres)
{
	std::srand(0x88e8fff4);

	for (auto i = 0U; i < num_spheres; ++i)
	{
		spheres[i].cx = RAND_FLOAT * 20.f - 10.f;
		spheres[i].cy = RAND_FLOAT * 20.f - 10.f;
		spheres[i].cz = RAND_FLOAT * 20.f - 5.f;
		spheres[i].radius = (RAND_FLOAT + 0.1f) * 1.5f;
		spheres[i].r = RAND_FLOAT;
		spheres[i].g = RAND_FLOAT;
		spheres[i].b = RAND_FLOAT;
	}
}


// Render the image img usign ray tracing for ortho projection camera.
// Each pixel of img contains color of closest sphere after the function has finished.
void trace(sphere const* spheres, std::uint32_t num_spheres, float* img)
{
	for (auto i = 0U; i < kImageWidth; ++i)
	{
		for (auto j = 0U; j < kImageHeight; ++j)
		{
			ray r;
			r.oz = NEAR;
			r.ox = LEFT + (WIDTH / kImageWidth) * (i + 0.5f);
			r.oy = BOTTOM + (HEIGHT / kImageHeight) * (j + 0.5f);
			r.dx = r.dy = 0.f;
			r.dz = 1.f;
			r.maxt = FAR - NEAR;

			int idx = -1;

			for (auto k = 0U; k < num_spheres; ++k)
			{
				if (intersect_sphere(spheres[k], r))
				{
					idx = k;
				}
			}

			if (idx != -1) // previous condition idx > 0 misses intersection with spheres[0]
			{
				img[(j * kImageWidth + i) * 3] = spheres[idx].r;
				img[(j * kImageWidth + i) * 3 + 1] = spheres[idx].g;
				img[(j * kImageWidth + i) * 3 + 2] = spheres[idx].b;
			}
			else
			{
				img[(j * kImageWidth + i) * 3] = 0.1f;
				img[(j * kImageWidth + i) * 3 + 1] = 0.1f;
				img[(j * kImageWidth + i) * 3 + 2] = 0.1f;
			}
		}
	}
}


void trace_volume_tree(VolumeTree tree, float* img, int width, int height)
{
	for (auto i = 0; i < width; ++i)
	{
		for (auto j = 0; j < height; ++j)
		{
			ray r;
			r.oz = NEAR;
			r.ox = LEFT + (WIDTH / width) * (i + 0.5f);
			r.oy = BOTTOM + (HEIGHT / height) * (j + 0.5f);
			r.dx = r.dy = 0.f;
			r.dz = 1.f;
			r.maxt = FAR - NEAR;

			sphere const* closest;
			if (tree.get_intersection(r, closest))
			{

				img[(j * width + i) * 3] = closest->r;
				img[(j * width + i) * 3 + 1] = closest->g;
				img[(j * width + i) * 3 + 2] = closest->b;
			}
			else
			{
				img[(j * width + i) * 3] = 0.1f;
				img[(j * width + i) * 3 + 1] = 0.1f;
				img[(j * width + i) * 3 + 2] = 0.1f;
			}
		}
	}
}

// A simple image magnification based on linear interpolation.
// Tested only on resolutions and magnification factors that are powers of 2.
std::vector<float> linear_magnification(float* original, int width, int height, int magnification_factor, int& new_width, int &new_height)
{
	new_width = width * magnification_factor;
	new_height = height * magnification_factor;

	std::vector<float> magnified_img(new_width * new_height * 3);

	// Iterating over 2x2 grids
	for (int i = 0; i < width ; i += 2)
	{
		for (int j = 0; j < height; j += 2)
		{
			int x0 = magnification_factor * i;
			int y0 = magnification_factor * j;
			int x1 = magnification_factor * i + magnification_factor * 2 - 1;
			int y1 = magnification_factor * j + magnification_factor * 2 - 1;

			float corners[2][2][3];

			corners[0][0][0] = original[(j * width + i) * 3];
			corners[0][0][1] = original[(j * width + i) * 3 + 1];
			corners[0][0][2] = original[(j * width + i) * 3 + 2];

			corners[0][1][0] = original[((j + 1) * width + i) * 3];
			corners[0][1][1] = original[((j + 1) * width + i) * 3 + 1];
			corners[0][1][2] = original[((j + 1) * width + i) * 3 + 2];

			corners[1][0][0] = original[(j * width + i + 1) * 3];
			corners[1][0][1] = original[(j * width + i + 1) * 3 + 1];
			corners[1][0][2] = original[(j * width + i + 1) * 3 + 2];

			corners[1][1][0] = original[((j + 1) * width + i + 1) * 3];
			corners[1][1][1] = original[((j + 1) * width + i + 1) * 3 + 1];
			corners[1][1][2] = original[((j + 1) * width + i + 1) * 3 + 2];
			
			// Edges
			for (int x = x0; x <= x1; ++x)
			{
				magnified_img[(y0 * new_width + x) * 3] = corners[0][0][0] + (x - x0) * (corners[1][0][0] - corners[0][0][0]) / (x1 - x0);
				magnified_img[(y0 * new_width + x) * 3 + 1] = corners[0][0][1] + (x - x0) * (corners[1][0][1] - corners[0][0][1]) / (x1 - x0);
				magnified_img[(y0 * new_width + x) * 3 + 2] = corners[0][0][2] + (x - x0) * (corners[1][0][2] - corners[0][0][2]) / (x1 - x0);

				magnified_img[(y1 * new_width + x) * 3] = corners[0][1][0] + (x - x0) * (corners[1][1][0] - corners[0][1][0]) / (x1 - x0);
				magnified_img[(y1 * new_width + x) * 3 + 1] = corners[0][1][1] + (x - x0) * (corners[1][1][1] - corners[0][1][1]) / (x1 - x0);
				magnified_img[(y1 * new_width + x) * 3 + 2] = corners[0][1][2] + (x - x0) * (corners[1][1][2] - corners[0][1][2]) / (x1 - x0);
			}

			for (int y = y0; y <= y1; ++y)
			{
				magnified_img[(y * new_width + x0) * 3] = corners[0][0][0] + (y - y0) * (corners[0][1][0] - corners[0][0][0]) / (y1 - y0);
				magnified_img[(y * new_width + x0) * 3 + 1] = corners[0][0][1] + (y - y0) * (corners[0][1][1] - corners[0][0][1]) / (y1 - y0);
				magnified_img[(y * new_width + x0) * 3 + 2] = corners[0][0][2] + (y - y0) * (corners[0][1][2] - corners[0][0][2]) / (y1 - y0);

				magnified_img[(y * new_width + x1) * 3] = corners[1][0][0] + (y - y0) * (corners[1][1][0] - corners[1][0][0]) / (y1 - y0);
				magnified_img[(y * new_width + x1) * 3 + 1] = corners[1][0][1] + (y - y0) * (corners[1][1][1] - corners[1][0][1]) / (y1 - y0);
				magnified_img[(y * new_width + x1) * 3 + 2] = corners[1][0][2] + (y - y0) * (corners[1][1][2] - corners[1][0][2]) / (y1 - y0);
			}

			// Inner pixels
			for (int x = x0 + 1; x < x1; ++x)
			{
				for (int y = y0 + 1; y < y1; ++y)
				{
					magnified_img[(y * new_width + x) * 3] = 
						(
							magnified_img[(y0 * new_width + x) * 3] * (y1 - y) 
							+ magnified_img[(y1 * new_width + x) * 3] * (y - y0)
						) 
						/ (y1 - y0);
					magnified_img[(y * new_width + x) * 3 + 1] =
						(
							magnified_img[(y0 * new_width + x) * 3 + 1] * (y1 - y)
							+ magnified_img[(y1 * new_width + x) * 3 + 1] * (y - y0)
							)
						/ (y1 - y0);
					magnified_img[(y * new_width + x) * 3 + 2] =
						(
							magnified_img[(y0 * new_width + x) * 3 + 2] * (y1 - y)
							+ magnified_img[(y1 * new_width + x) * 3 + 2] * (y - y0)
							)
						/ (y1 - y0);
				}
			}
		}
	}

	return magnified_img;
}

bool save_img(std::vector<float> const& img, int width, int height, const char* file_name)
{
	OIIO_NAMESPACE_USING;

	ImageOutput* out = ImageOutput::create(file_name);

	if (!out)
	{
		return false;
	}

	ImageSpec spec2(width, height, 3, TypeDesc::FLOAT);

	out->open(file_name, spec2);
	out->write_image(TypeDesc::FLOAT, &img[0], sizeof(float) * 3);
	out->close();

	return true;
}

int main()
{
	std::vector<sphere> spheres(kNumSpheres);
	generate_spheres(&spheres[0], kNumSpheres);

	//--------------------------------Brut-Forced------------------------------------------------------
	std::vector<float> img(kImageWidth * kImageHeight * 3);

	auto start = std::chrono::high_resolution_clock::now();
	trace(&spheres[0], kNumSpheres, &img[0]);
	auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();

	std::cout << "Execution time: " << delta << " ms\n";

	if (!save_img(img, kImageWidth, kImageHeight, "result.png"))
	{
		std::cout << "Can't create image file on disk";
		return -1;
	}

	//--------------------------------With-Volume-Tree-------------------------------------------------
	VolumeTree tree;
	tree.build(&spheres[0], kNumSpheres);
	std::vector<float> img2(kImageWidth * kImageHeight * 3);

	start = std::chrono::high_resolution_clock::now();
	trace_volume_tree(tree, &img2[0], kImageWidth, kImageHeight);
	delta = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();

	std::cout << "Execution time with volume tree: " << delta << " ms\n";

	if (!save_img(img2, kImageWidth, kImageHeight, "volume_tree.png"))
	{
		std::cout << "Can't create image file on disk";
		return -1;
	}


	//--------------------------------With-Volume-Tree-And-Magnification-------------------------------
	int magnification_factor = 2;
	int width = kImageWidth / magnification_factor;
	int height = kImageHeight / magnification_factor;
	std::vector<float> img3(width * height * 3);

	start = std::chrono::high_resolution_clock::now();
	trace_volume_tree(tree, &img3[0], width, height);
	delta = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();

	std::cout << "Execution time with volume tree and magnification: " << delta << " ms\n";
	
	int new_width, new_height;
	auto magnified = linear_magnification(&img3[0], width, height, magnification_factor, new_width, new_height);
	if (!save_img(magnified, new_width, new_height, "volume_tree_magnification.png"))
	{
		std::cout << "Can't create image file on disk";
		return -1;
	}

	return 0;
}
