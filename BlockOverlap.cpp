/*
	BlockOverlap plugin for Avisynth 2.5
	Version 0.1
	Copyright (C)2005 Alexander G. Balakhnin aka Fizick.
	bag@hotmail.ru         http://bag.hotmail.ru
	under the GNU General Public Licence version 2.

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

	This plugin takes two input clips with shifted blocks positions (by half of block diagonal)
	and produces deblocked output by blocks windowed overlapping (uniform or kernel non-uniform blending).
	It is useful for MVTools block motion compensation.

*/

#include "windows.h"
#include "include\avisynth.h"
#include "math.h"

/****************************
 * The following is the header definitions.
 * For larger projects, move this into a .h file
 * that can be included.
 ****************************/


class BlockOverlap : public GenericVideoFilter {
	// BlockOverlap defines the name of your filter class. 
	// This name is only used internally, and does not affect the name of your filter or similar.
	// This filter extends GenericVideoFilter, which incorporates basic functionality.
	// All functions present in the filter must also be present here.

	PClip shifted; // second clip, with blocks shifted by half blocksizes along diagonal 
	int xblksize;   // horizontal block size 
	int yblksize;   // vertical block size 
	float kernel; // weighting window form (0 - uniform, 1.0 - cosine with peak at block center)

	int* win; // luma weigthing window
	int* winUV; // weigthing window for U,V chroma planes
	int* winYUY2; //weigthing window combined
	int xblksizeUV;   // horizontal block size UV
	int yblksizeUV;   // vertical block size UV

public:
	// This defines that these functions are present in your class.
	// These functions must be that same as those actually implemented.
	// Since the functions are "public" they are accessible to other classes.
	// Otherwise they can only be called from functions within the class itself.

	BlockOverlap(PClip _child, PClip _shifted, int _xblksize, int _yblksize,
		float _kernel, IScriptEnvironment* env);
	// This is the constructor. It does not return any value, and is always used, 
	//  when an instance of the class is created.
	// Since there is no code in this, this is the definition.

	~BlockOverlap();
	// The is the destructor definition. This is called when the filter is destroyed.


	PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
	// This is the function that AviSynth calls to get a given frame.
	// So when this functions gets called, the filter is supposed to return frame n.
};

/***************************
 * The following is the implementation
 * of the defined functions.
 ***************************/

 //Here is the actual constructor code used
BlockOverlap::BlockOverlap(PClip _child, PClip _shifted, int _xblksize, int _yblksize,
	float _kernel, IScriptEnvironment* env) :
	GenericVideoFilter(_child), shifted(_shifted), xblksize(_xblksize), yblksize(_yblksize), kernel(_kernel) {
	// This is the implementation of the constructor.
	// The child clip (source clip) is inherited by the GenericVideoFilter,
	//  where the following variables gets defined:
	//   PClip child;   // Contains the source clip.
	//   VideoInfo vi;  // Contains videoinfo on the source clip.

	const float pi = 3.1415926535897932384626433832795f;

	if (vi.IsYV12()) {
		xblksizeUV = xblksize / 2;
		yblksizeUV = yblksize / 2; // for YV12
	}
	else if (vi.IsYUY2()) {
		xblksizeUV = xblksize / 2;
		yblksizeUV = yblksize; // for YUY2
	}
	else
		env->ThrowError("BlockOverlap: input must be YV12 or YUY2!");

	if (xblksize % 2 || yblksize % 2)
		env->ThrowError("BlockOverlap: block sizes must be even !");

	if (!vi.IsSameColorspace(shifted->GetVideoInfo()))
		env->ThrowError("BlockOverlap: shifted clip must have same color format!");


	win = (int*)malloc(xblksize * yblksize * sizeof(int));
	winUV = (int*)malloc(xblksizeUV * yblksizeUV * sizeof(int));
	winYUY2 = (int*)malloc(2 * xblksize * yblksize * sizeof(int));

	int* pwin = win;
	for (int h = 0; h < yblksize; h++)
	{
		float wy = cosf(pi * (h - yblksize / 2 + 0.5f) / yblksize);
		wy = wy * wy;
		for (int i = 0; i < xblksize; i++)
		{
			float wx = cosf(pi * (i - xblksize / 2 + 0.5f) / xblksize);
			wx = wx * wx;// left window (rised cosine)
			pwin[i] = int(((wx + wy) * kernel + (1 - kernel)) * 128); // window scaled to 256 to be integer
		}
		pwin += xblksize;
	}

	pwin = winUV;
	for (int h = 0; h < yblksizeUV; h++)
	{
		float wy = cosf(pi * (h - yblksizeUV / 2 + 0.5f) / yblksizeUV);
		wy = wy * wy;
		for (int i = 0; i < xblksizeUV; i++)
		{
			float wx = cosf(pi * (i - xblksizeUV / 2 + 0.5f) / xblksizeUV);
			wx = wx * wx;// left window (rised cosine)
			pwin[i] = int(((wx + wy) * kernel + (1 - kernel)) * 128); // window scaled to 256 max to be integer
		}
		pwin += xblksizeUV;
	}

	if (vi.IsYUY2())
	{
		pwin = win;
		int* pwinYUY2 = winYUY2;
		int* pwinUV = winUV;
		for (int h = 0; h < yblksize; h++)
		{
			for (int w = 0; w < xblksize * 2; w += 4)
			{
				pwinYUY2[w] = pwin[w >> 1];
				pwinYUY2[w + 1] = pwinUV[w >> 2];
				pwinYUY2[w + 2] = pwin[(w >> 1) + 1];
				pwinYUY2[w + 3] = pwinUV[w >> 2];

			}
			pwin += xblksize;
			pwinUV += xblksizeUV;
			pwinYUY2 += xblksize * 2;
		}

	}

}

// This is where any actual destructor code used goes
BlockOverlap::~BlockOverlap() {
	// This is where you can deallocate any memory you might have used.
	free(win);
	free(winUV);
	free(winYUY2);
}


PVideoFrame __stdcall BlockOverlap::GetFrame(int n, IScriptEnvironment* env) {

	int* pwin;
	int h, w, bx, by, nbx, nby;

	PVideoFrame sh = shifted->GetFrame(n, env);
	PVideoFrame src = child->GetFrame(n, env);
	env->MakeWritable(&src); // inplace

	if (vi.IsYV12())
	{
		int plane = PLANAR_Y;
		unsigned char* srcp = src->GetWritePtr(plane);
		int src_pitch = src->GetPitch(plane);
		int src_width = src->GetRowSize(plane);
		int src_height = src->GetHeight(plane);
		const unsigned char* shp = sh->GetReadPtr(plane);
		int sh_pitch = sh->GetPitch(plane);

		nbx = src_width / xblksize;
		nby = src_height / yblksize;

		for (by = 0; by < nby; by++)
		{
			pwin = win;
			for (h = 0; h < yblksize; h++)
			{
				for (bx = 0; bx < nbx; bx++)
				{
					for (w = 0; w < xblksize; w += 2)
					{
						srcp[w] = (srcp[w] * pwin[w] + shp[w] * (256 - pwin[w])) >> 8;
						srcp[w + 1] = (srcp[w + 1] * pwin[w + 1] + shp[w + 1] * (256 - pwin[w + 1])) >> 8;
					}
					srcp += xblksize;
					shp += xblksize;

				}
				pwin += xblksize;
				srcp += src_pitch - xblksize * nbx;
				shp += sh_pitch - xblksize * nbx;
			}
		}

		plane = PLANAR_U;
		srcp = src->GetWritePtr(plane);
		src_pitch = src->GetPitch(plane);
		src_width = src->GetRowSize(plane);
		src_height = src->GetHeight(plane);
		shp = sh->GetReadPtr(plane);
		sh_pitch = sh->GetPitch(plane);

		nbx = src_width / xblksizeUV;
		nby = src_height / yblksizeUV;

		for (by = 0; by < nby; by++)
		{
			pwin = winUV;
			for (h = 0; h < yblksizeUV; h++)
			{
				for (bx = 0; bx < nbx; bx++)
				{
					for (w = 0; w < xblksizeUV; w++)
					{
						srcp[w] = (srcp[w] * pwin[w] + shp[w] * (256 - pwin[w])) >> 8;
					}
					srcp += xblksizeUV;
					shp += xblksizeUV;

				}
				pwin += xblksizeUV;
				srcp += src_pitch - xblksizeUV * nbx;
				shp += sh_pitch - xblksizeUV * nbx;
			}
		}

		plane = PLANAR_V;
		srcp = src->GetWritePtr(plane);
		src_pitch = src->GetPitch(plane);
		src_width = src->GetRowSize(plane);
		src_height = src->GetHeight(plane);
		shp = sh->GetReadPtr(plane);
		sh_pitch = sh->GetPitch(plane);

		nbx = src_width / xblksizeUV;
		nby = src_height / yblksizeUV;

		for (by = 0; by < nby; by++)
		{
			pwin = winUV;
			for (h = 0; h < yblksizeUV; h++)
			{
				for (bx = 0; bx < nbx; bx++)
				{
					for (w = 0; w < xblksizeUV; w++)
					{
						srcp[w] = (srcp[w] * pwin[w] + shp[w] * (256 - pwin[w])) >> 8;
					}
					srcp += xblksizeUV;
					shp += xblksizeUV;

				}
				pwin += xblksizeUV;
				srcp += src_pitch - xblksizeUV * nbx;
				shp += sh_pitch - xblksizeUV * nbx;
			}
		}
	}
	else if (vi.IsYUY2())
	{
		unsigned char* srcp = src->GetWritePtr();
		int src_pitch = src->GetPitch();
		int src_width = src->GetRowSize();
		int src_height = src->GetHeight();
		const unsigned char* shp = sh->GetReadPtr();
		int sh_pitch = sh->GetPitch();

		nbx = src_width / xblksize;
		nby = src_height / yblksize;

		for (by = 0; by < nby; by++)
		{
			pwin = winYUY2;
			for (h = 0; h < yblksize; h++)
			{
				for (bx = 0; bx < nbx; bx++)
				{
					for (w = 0; w < xblksize * 2; w += 2)
					{
						srcp[w] = (srcp[w] * pwin[w] + shp[w] * (256 - pwin[w])) >> 8;
						srcp[w + 1] = (srcp[w + 1] * pwin[w + 1] + shp[w + 1] * (256 - pwin[w + 1])) >> 8;
					}
					srcp += xblksize;
					shp += xblksize;

				}
				pwin += xblksize * 2;
				srcp += src_pitch - xblksize * nbx;
				shp += sh_pitch - xblksize * nbx;
			}
		}
	}


	return src;
}


// This is the function that created the filter, when the filter has been called.
// This can be used for simple parameter checking, so it is possible to create different filters,
// based on the arguments recieved.

AVSValue __cdecl Create_BlockOverlap(AVSValue args, void* user_data, IScriptEnvironment* env) {
	return new BlockOverlap(args[0].AsClip(), // the 0th parameter is the source clip
		args[1].AsClip(), // Corresponds to our 1st parameter - shifted clip 
		args[2].AsInt(8), // Corresponds to our 2nd parameter - horizontal block size
		args[3].AsInt(8), // Corresponds to our 3nd parameter - vertical block size
		(float)args[4].AsFloat(0.5), // Corresponds to our 3nd parameter - window kernel factor
		env);
	// Calls the constructor with the arguments provied.
}


// The following function is the function that actually registers the filter in AviSynth
// It is called automatically, when the plugin is loaded to see which functions this filter contains.
const AVS_Linkage* AVS_linkage = 0;

//extern "C" __declspec(dllexport) const char* __stdcall AvisynthPluginInit2(IScriptEnvironment * env) 
extern "C" __declspec(dllexport) const char* __stdcall AvisynthPluginInit3(IScriptEnvironment * env, const AVS_Linkage* const vectors)
{
	AVS_linkage = vectors;
	env->AddFunction("BlockOverlap", "c[shifted]c[xblksize]i[yblksize]i[kernel]f", Create_BlockOverlap, 0);
	// The AddFunction has the following parameters:
	// AddFunction(Filtername , Arguments, Function to call,0);

	// Arguments is a string that defines the types and optional names of the arguments for you filter.
	// c - Video Clip
	// i - Integer number
	// f - Float number
	// s - String
	// b - boolean

	 // The word inside the [ ] lets you used named parameters in your script

	return "`BlockOverlap' BlockOverlap plugin by Fizick";
	// A freeform name of the plugin.
}

