#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <xmmintrin.h>
#include <emmintrin.h>
#include <pmmintrin.h>
#include <tmmintrin.h>

#include <cv.h>
#include "labelling.h"
#include "threshold.h"
#include "integral-image.h"

#define label_aliases_index( arr, index ) g_array_index( arr, label_t, index )
#define label_clips_index( arr, index ) g_array_index( arr, koki_clip_region_t, index )

static label_t label_find_canonical( koki_labelled_image_t * restrict lmg,
				     label_t l )
{
	while(1) {
		label_t a = label_aliases_index( lmg->aliases, l-1 );

		if( a == l )
			/* Found the lowest alias */
			return a;

		l = a;
	}
}

static void label_alias( koki_labelled_image_t * restrict lmg, label_t l_canon, label_t l_alias )
{
	label_t * restrict l;

	/* Resolve to the minimum alias of l_alias */
	l_alias = label_find_canonical( lmg, l_alias );
	l_canon = label_find_canonical( lmg, l_canon );

	/* Alias l_alias to l_canon */
	l = &label_aliases_index( lmg->aliases, l_alias-1 );
	*l = l_canon;
}

#define wrapped_old_label_access(base, idx, width) (&(base)[((idx) >= (width)+1) ? ((idx) - ((width)+1)) : (idx)])

static void set_label(koki_labelled_image_t * restrict labelled_image,
		      uint16_t x, uint16_t y, label_t label,
		      label_t * restrict old_labels,
		      unsigned int * restrict old_label_idx)
{

	if (label == 0){
		;
	} else {
		koki_clip_region_t * restrict clip;

		assert(labelled_image->aliases != NULL
		       && labelled_image->aliases->len > label-1);

		clip = &label_clips_index( labelled_image->clips, label-1 );
		clip->mass++;
		clip->max.x = MAX(clip->max.x, x);
		clip->max.y = MAX(clip->max.y, y);
		clip->min.x = MIN(clip->min.x, x);
		clip->min.y = MIN(clip->min.y, y);

		if (clip->min.y == y)
			clip->top_most_x = x;

		/* Label image as either zero or one, for light or dark. */
		*labelled_image->labelling_cur_ptr |=
			(1 << labelled_image->labelling_counter);
	}

	*wrapped_old_label_access(old_labels, *old_label_idx, labelled_image->w)
		= label;

	(*old_label_idx)++;
	if ((*old_label_idx) >= (labelled_image->w + 1))
		*old_label_idx = 0;

	if (++labelled_image->labelling_counter == 32) {
		labelled_image->labelling_cur_ptr++;
		*labelled_image->labelling_cur_ptr = 0;
		labelled_image->labelling_counter = 0;
	}
}

static label_t get_old_label(label_t * restrict old_labels,
					unsigned int * restrict old_label_idx,
					unsigned int width,
					enum DIRECTION direction)
{
	int offs = 0;
	switch (direction) {
	case N:
		offs = 1;
		break;
	case NE:
		offs = 2;
		break;
	case NW:
		offs = 0;
		break;
	case W:
		offs = width; /* Will wrap */
		break;
	default:
		abort();
	}

	offs += *old_label_idx;
	return *wrapped_old_label_access(old_labels, offs, width);
}

static void label_dark_pixel( koki_labelled_image_t * restrict lmg,
			      uint16_t x, uint16_t y,
			      label_t * restrict old_labels,
			      unsigned int * restrict old_label_idx)
{
	label_t label_tmp;

	/* if pixel above is labelled, join that label */
	label_tmp = get_old_label(old_labels, old_label_idx, lmg->w, N);
	if (label_tmp > 0){
		set_label(lmg, x, y, label_tmp, old_labels, old_label_idx);
		return;
	}

	/* if NE pixel is labelled, some merging may need to occur */
	label_tmp = get_old_label(old_labels, old_label_idx, lmg->w, NE);
	if (label_tmp > 0){

		label_t label_w, label_nw, l1, l2, label_min, label_max;
		label_w = get_old_label(old_labels, old_label_idx, lmg->w, W);
		label_nw = get_old_label(old_labels, old_label_idx, lmg->w, NW);

		/* if one of the pixels W or NW are labelled, they should
		   be merged together */
		if ((label_w > 0 || label_nw > 0) && x != 0) {

			l1 = label_aliases_index( lmg->aliases, label_tmp-1 );

			l2 = label_nw > 0
				? label_aliases_index( lmg->aliases, label_nw-1 )
				: label_aliases_index( lmg->aliases, label_w-1 );

			/* identify lowest label */
			label_max = l2;
			label_min = l1;
			if (label_max < label_min){
				label_min = l2;
				label_max = l1;
			}

			set_label(lmg, x, y, label_min, old_labels,
					old_label_idx);
			label_alias( lmg, label_min, label_max );
		} else {

			set_label(lmg, x, y, label_tmp, old_labels,
					old_label_idx);

		}

		return;
	}

	/* Otherwise, take the NW label, if present */
	label_tmp = get_old_label(old_labels, old_label_idx, lmg->w, NW);
	if (label_tmp > 0 && x != 0) {
		set_label(lmg, x, y, label_tmp, old_labels, old_label_idx);
		return;
	}

	/* Otherwise, take the W label, if present */
	label_tmp = get_old_label(old_labels, old_label_idx, lmg->w, W);
	if (label_tmp > 0 && x != 0) {
		set_label(lmg, x, y, label_tmp, old_labels, old_label_idx);
		return;
	}

	/* If we get this far, a new region has been found */

	/* Check we do not exceed the maximum number of labels */
	assert( lmg->aliases->len != KOKI_LABEL_MAX );

	label_tmp = lmg->aliases->len + 1;
	g_array_append_val(lmg->aliases, label_tmp);

	/* Init clip for this label. Set the max and minimum pixel locations to
	 * this pixels location. */
	koki_clip_region_t clip;
	clip.mass = 0;
	clip.max.x = x;
	clip.max.y = y;
	clip.min.x = x;
	clip.min.y = y;
	clip.top_least_x = x;
	clip.top_most_x = x;
	g_array_append_val(lmg->clips, clip);

	set_label(lmg, x, y, label_tmp, old_labels, old_label_idx);
}

static void label_image_calc_stats( koki_labelled_image_t * restrict labelled_image )
{
	/* Now renumber all labels to ensure they're all canonical */
	for( label_t i=1; i<labelled_image->aliases->len; i++ ) {
		label_t * restrict a = &label_aliases_index( labelled_image->aliases, i-1 );

		*a = label_find_canonical( labelled_image, i );
	}

	/* collect label statistics (mass, bounding box) */

	label_t max_alias = 0;
	GArray * restrict aliases, * restrict clips;

	aliases = labelled_image->aliases;
	clips = labelled_image->clips;

	/* Iterate over all of the aliases, merging their clips bounds into the
	 * aliased clip. Output is an array of clips, with accumulated bounds
	 * if the indexes corresponding label wasn't an alias, with all fields
	 * zero if it was. */
	for( label_t i=1; i<labelled_image->aliases->len; i++ ) {
		koki_clip_region_t * restrict alias_clip, * restrict canonical_clip;

		label_t canonical_label =
			label_aliases_index( labelled_image->aliases, i-1 );

		/* Merge this aliases clip into the aliased labels clip */
		alias_clip = &label_clips_index( clips, i-1 );
		canonical_clip = &label_clips_index( clips, canonical_label-1 );

		if (i == canonical_label)
			/* This label is not an alias, therefore its clip
			 * doesn't need to be merged into anything. */
			continue;

		/* First, merge topmost pixel record. Whichever clip has the
		 * minimum y value, pick its topmost pixel. Except where they're
		 * the same, in which case merge them. */
		if (canonical_clip->min.y > alias_clip->min.y) {
			/* Replace */
			canonical_clip->top_least_x = alias_clip->top_least_x;
			canonical_clip->top_most_x = alias_clip->top_most_x;
		} else if (canonical_clip->min.y == alias_clip->min.y) {
			/* Merge. Indentation recommendations welcome. */
			canonical_clip->top_least_x =
				MIN(alias_clip->top_least_x,
				    canonical_clip->top_least_x);
			canonical_clip->top_most_x =
				MIN(alias_clip->top_most_x,
				    canonical_clip->top_most_x);
		}

		canonical_clip->mass += alias_clip->mass;
		canonical_clip->max.x =
			MAX(canonical_clip->max.x, alias_clip->max.x);
		canonical_clip->max.y =
			MAX(canonical_clip->max.y, alias_clip->max.y);
		canonical_clip->min.x =
			MIN(canonical_clip->min.x, alias_clip->min.x);
		canonical_clip->min.y =
			MIN(canonical_clip->min.y, alias_clip->min.y);

		/* Invalidate the alias clip's values, so that it's ignored by
		 * all other code. */
		alias_clip->mass = 0;
		alias_clip->max.x = 0;
		alias_clip->max.y = 0;
		alias_clip->min.x = 0xFFFF;
		alias_clip->min.y = 0xFFFF;
	}
}

static uint32_t koki_funky_integral_image_sum( const uint32_t *ii, int imgwidth,
		const CvRect *region );

static bool
koki_funky_threshold_adaptive_pixel( const IplImage * restrict frame,
				    uint32_t * restrict iimg,
				    const CvRect * restrict roi,
				    uint16_t x, uint16_t y, int16_t c )
{
	uint16_t w, h;
	uint32_t sum;
	uint32_t cmp;

	w = roi->width;
	h = roi->height;

	/* calculate threshold */
	sum = koki_funky_integral_image_sum( iimg, frame->width, roi );

	/* The following is a rearranged version of
	      threshold = sum / (w*h);
	      if( KOKI_IPLIMAGE_GS_ELEM(frame, x, y) > (threshold-c) ) ...
	   This is re-arranged to avoid division. */

	cmp = KOKI_IPLIMAGE_GS_ELEM(frame, x, y) + c;
	cmp *= w * h;

	/* apply threshold */
	if( cmp > sum )
		return true;

	return false;
}

#define ii_pix( img, w, x, y ) ( (img)[(((y) % 16) * w) + (x)])

static void
koki_funky_integral_image_advance( uint32_t * restrict ii, int imgwidth,
		                   const IplImage * restrict srcimg,
				   uint32_t * restrict sum, uint16_t width,
				   int y)
{
	uint16_t x;

	uint32_t * restrict span = &ii[(y % 16) * imgwidth];
	uint32_t * restrict srcspan = (uint32_t *)((srcimg)->imageData + (srcimg)->widthStep*(y));
	uint32_t v = 0;
	//assert((width & 3) == 0);

	__m128i accuml_horizontal = _mm_set1_epi32(0);

	for( x=0; x < width; x += 4) {
		/* Load some source data as bytes, and unpack to 32 bit dwords*/
		__m128i databytes =  _mm_set1_epi32(*srcspan++);
		__m128i zeroreg = _mm_cvtsi32_si128(0);
		__m128i datawords = _mm_unpackhi_epi8(databytes, zeroreg);
		__m128i datadwords = _mm_unpackhi_epi16(datawords, zeroreg);

		/* Load the sum data */
		__m128i column_data = _mm_load_si128(sum);

		/* Accumulate sum data and store */
		__m128i summed_columns = _mm_add_epi32(column_data, datadwords);
		_mm_store_si128(sum, summed_columns);
		sum += 4;

		/* Add in the horizontal value accumulated this far. */
		__m128i summed_results =
			_mm_add_epi32(summed_columns, accuml_horizontal);

		/* Complicated: sum dwords horizontally. We can do this with
		 * pairs, but not across the whole vector. */
		/* The result here is 0, 0, a+b, c+d. */
		__m128i horiz_sum_pair = _mm_hadd_epi32(zeroreg,summed_results);

		/* Result: a, b, abc, ??? */
		__m128i foobar = _mm_add_epi32(summed_results, horiz_sum_pair);

		/* Result: abcd, 0, 0, 0 */
		__m128i qux = _mm_hadd_epi32(summed_results, zeroreg);
		qux = _mm_hadd_epi32(qux, zeroreg);

		/* Result: abcd, 0, ab, cd */
		qux = _mm_or_si128(qux, horiz_sum_pair);

		/* Now, foobar and qux contain two pieces of final data each.
		 * Shuffle them all into the same vector. The selection is:
		 * dword 0 from foobar -> 0
		 * dword 2 from foobar -> 2
		 * dword 0 from qux -> 0
		 * dword 2 from qux -> 2
		 * Giving us the result: a, abc, abcd, ab */
		__m128i final = (__m128i)_mm_shuffle_ps((__m128)foobar, (__m128)qux, 0x88);

		/* And finally, reshuffle into the useful order. We want:
		 * a:    dword 0
		 * ab:   dword 3
		 * abc:  dword 1
		 * abcd: dword 2
		 * Resulting in a, ab, abc, abcd */
		final = _mm_shuffle_epi32(final, 0x9C);

		/* Store back. */
		_mm_store_si128(span, final);

		/* Pick out the final accumulated value and store it, so that
		 * we can add it to the next part of the line */
		span += 3;
		accuml_horizontal = _mm_cvtsi32_si128(*span++);
	}
}

static uint32_t
koki_funky_integral_image_sum( const uint32_t * restrict ii, int imgwidth,
				  const CvRect * restrict region )
{
	uint32_t v;
	/* Coordinates of the south-east pixel of the region */
	const uint32_t se_x = region->x + region->width - 1;
	const uint32_t se_y = region->y + region->height - 1;

	/* SE corner */
	v = ii_pix( ii, imgwidth, se_x, se_y );

	if( region->x > 0 && region->y > 0 )
		/* NW of top left corner */
		v += ii_pix( ii, imgwidth, region->x - 1, region->y - 1 );

	if( region->x > 0 )
		/* E of bottom left corner */
		v -= ii_pix( ii, imgwidth, region->x - 1, se_y );

	if( region->y > 0 )
		/* N of top right corner */
		v -= ii_pix( ii, imgwidth, se_x, region->y - 1 );

	return v;
}

koki_labelled_image_t* koki_funky_label_adaptive( koki_t * restrict koki,
					    const IplImage * restrict frame,
					    uint16_t window_size,
					    int16_t thresh_margin )
{
	label_t old_labels[frame->width+1]; /* NB: c99 var-sized array */
	uint16_t x, y;
	koki_labelled_image_t *lmg;
	unsigned int old_label_idx = 0;

	memset(old_labels, 0, sizeof(old_labels));

	/* Instead of an integral image, use a plain array instead. We can
	 * then optimise around this later. Also: the sum array. */
	uint32_t * restrict iimg =
		calloc(1, sizeof(uint32_t) * frame->width * 16);
	uint32_t * restrict sumarr = calloc(1, sizeof(uint32_t) * frame->width);

	assert(frame != NULL && frame->nChannels == 1);

	lmg = koki_labelled_image_new( frame->width, frame->height );
	lmg->labelling_cur_ptr = lmg->data;
	*lmg->labelling_cur_ptr = 0;
	lmg->labelling_counter = 0;

	CvRect win;
	int yadvance = window_size / 2;

	/* Process the first lump of the image */
	for (int i = 0; i < yadvance; i++)
		koki_funky_integral_image_advance(iimg, frame->width,
						   frame, sumarr,
						   frame->width - 1, i);

	int half_win_size = window_size / 2;
	int winx, winy, winwidth, winheight;
	winy = -half_win_size;
	winheight = 1;
	for( y=0; y<frame->height; y++ ) {

		/* Advance the integral image */
		yadvance++;
		yadvance = MIN(yadvance, frame->height - 1);

		/* Skip repeatedly integrating the last line */
		if (yadvance != frame->height || winheight < window_size)
			koki_funky_integral_image_advance( iimg, frame->width,
					frame, sumarr, frame->width - 1,
					yadvance);

		winx = -half_win_size;
		winwidth = 1;

		for( x=0; x<frame->width; x++ ) {
			win.x = MAX(0, winx);
			win.y = MAX(0, winy);
			win.width = winwidth;
			win.height = winheight;

			if( koki_funky_threshold_adaptive_pixel( frame,
							   iimg, &win,
							   x, y, thresh_margin ) ) {
				/* Nothing exciting */
				set_label( lmg, x, y, 0, old_labels,
						&old_label_idx );
			} else {
				/* Label the thing */
				label_dark_pixel( lmg, x, y, old_labels,
						&old_label_idx );
			}

			winwidth = MIN(winwidth + 1, window_size);
			winx = MIN(frame->width, winx+1);
		}

		winheight = MIN(winheight + 1, window_size);
		winy = MIN(frame->height, winy+1);
	}

	/* Sort out all the remaining labelling related stuff */
	label_image_calc_stats( lmg );

	free(iimg);
	free(sumarr);

	return lmg;
}
