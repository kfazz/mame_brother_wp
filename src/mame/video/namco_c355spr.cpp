// license:BSD-3-Clause
// copyright-holders:David Haywood, Phil Stroffolino

/*
    C355 Zooming sprites
    used by
    namcofl.cpp (all games)
    namconb1.cpp (all games)
    gal3.cpp (all games)
    namcos21.cpp (Driver's Eyes, Solvalou, Starblade, Air Combat, Cyber Sled) (everything except Winning Run series)
    namcos2.cpp (Steel Gunner, Steel Gunner 2, Lucky & Wild, Suzuka 8 Hours, Suzuka 8 Hours 2)

*/

#include "emu.h"
#include "namco_c355spr.h"

DEFINE_DEVICE_TYPE(NAMCO_C355SPR, namco_c355spr_device, "namco_c355spr", "Namco C355 (Sprites)")

namco_c355spr_device::namco_c355spr_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, NAMCO_C355SPR, tag, owner, clock),
	m_gfx_region(0),
	m_palxor(0),
	m_is_namcofl(false),
	m_gfxdecode(*this, finder_base::DUMMY_TAG),
	m_palette(*this, finder_base::DUMMY_TAG)
{
}


/**************************************************************************************/

void namco_c355spr_device::zdrawgfxzoom(
		screen_device &screen,
		bitmap_ind16 &dest_bmp,const rectangle &clip,gfx_element *gfx,
		uint32_t code,uint32_t color,int flipx,int flipy,int sx,int sy,
		int scalex, int scaley, int zpos )
{
	if (!scalex || !scaley) return;
	if (dest_bmp.bpp() == 16)
	{
		if( gfx )
		{
			int shadow_offset = (m_palette->shadows_enabled())?m_palette->entries():0;
			const pen_t *pal = &m_palette->pen(gfx->colorbase() + gfx->granularity() * (color % gfx->colors()));
			const uint8_t *source_base = gfx->get_data(code % gfx->elements());
			int sprite_screen_height = (scaley*gfx->height()+0x8000)>>16;
			int sprite_screen_width = (scalex*gfx->width()+0x8000)>>16;
			if (sprite_screen_width && sprite_screen_height)
			{
				/* compute sprite increment per screen pixel */
				int dx = (gfx->width()<<16)/sprite_screen_width;
				int dy = (gfx->height()<<16)/sprite_screen_height;

				int ex = sx+sprite_screen_width;
				int ey = sy+sprite_screen_height;

				int x_index_base;
				int y_index;

				if( flipx )
				{
					x_index_base = (sprite_screen_width-1)*dx;
					dx = -dx;
				}
				else
				{
					x_index_base = 0;
				}

				if( flipy )
				{
					y_index = (sprite_screen_height-1)*dy;
					dy = -dy;
				}
				else
				{
					y_index = 0;
				}

				if( sx < clip.min_x)
				{ /* clip left */
					int pixels = clip.min_x-sx;
					sx += pixels;
					x_index_base += pixels*dx;
				}
				if( sy < clip.min_y )
				{ /* clip top */
					int pixels = clip.min_y-sy;
					sy += pixels;
					y_index += pixels*dy;
				}
				if( ex > clip.max_x+1 )
				{ /* clip right */
					int pixels = ex-clip.max_x-1;
					ex -= pixels;
				}
				if( ey > clip.max_y+1 )
				{ /* clip bottom */
					int pixels = ey-clip.max_y-1;
					ey -= pixels;
				}

				if( ex>sx )
				{ /* skip if inner loop doesn't draw anything */
					int y;
					bitmap_ind8 &priority_bitmap = screen.priority();
					if( priority_bitmap.valid() )
					{
						for( y=sy; y<ey; y++ )
						{
							const uint8_t *source = source_base + (y_index>>16) * gfx->rowbytes();
							uint16_t *dest = &dest_bmp.pix16(y);
							uint8_t *pri = &priority_bitmap.pix8(y);
							int x, x_index = x_index_base;
							if( m_palxor )
							{
								for( x=sx; x<ex; x++ )
								{
									int c = source[x_index>>16];
									if( c != 0xff )
									{
										if( pri[x]<=zpos )
										{
											switch( c )
											{
											case 0:
												dest[x] = 0x4000|(dest[x]&0x1fff);
												break;
											case 1:
												dest[x] = 0x6000|(dest[x]&0x1fff);
												break;
											default:
												dest[x] = pal[c];
												break;
											}
											pri[x] = zpos;
										}
									}
									x_index += dx;
								}
								y_index += dy;
							}
							else
							{
								for( x=sx; x<ex; x++ )
								{
									int c = source[x_index>>16];
									if( c != 0xff )
									{
										if( pri[x]<=zpos )
										{
											if( color == 0xf && c==0xfe && shadow_offset )
											{
												dest[x] |= shadow_offset;
											}
											else
											{
												dest[x] = pal[c];
											}
											pri[x] = zpos;
										}
									}
									x_index += dx;
								}
								y_index += dy;
							}
						}
					}
				}
			}
		}
	}
} /* zdrawgfxzoom */

void namco_c355spr_device::zdrawgfxzoom(
		screen_device &screen,
		bitmap_rgb32 &dest_bmp,const rectangle &clip,gfx_element *gfx,
		uint32_t code,uint32_t color,int flipx,int flipy,int sx,int sy,
		int scalex, int scaley, int zpos )
{
	/* nop */
}

void namco_c355spr_device::device_start()
{
	//m_spriteram.resize(m_ramsize);
	std::fill(std::begin(m_spriteram), std::end(m_spriteram), 0x0000);  // needed for Nebulas Ray
	std::fill(std::begin(m_position), std::end(m_position), 0x0000);

	save_item(NAME(m_spriteram));
	save_item(NAME(m_position));
}


/**************************************************************************************/

WRITE16_MEMBER( namco_c355spr_device::position_w )
{
	COMBINE_DATA(&m_position[offset]);
}
READ16_MEMBER( namco_c355spr_device::position_r )
{
	return m_position[offset];
}

/**************************************************************************************************************/

/**
 * 0x00000 sprite attr (page0)
 * 0x02000 sprite list (page0)
 *
 * 0x02400 window attributes
 * 0x04000 format
 * 0x08000 tile
 * 0x10000 sprite attr (page1)
 * 0x14000 sprite list (page1)
 */
template<class _BitmapClass>
void namco_c355spr_device::draw_sprite(screen_device &screen, _BitmapClass &bitmap, const rectangle &cliprect, const uint16_t *pSource, int pri, int zpos )
{
	uint16_t *spriteram16 = m_spriteram;
	unsigned screen_height_remaining, screen_width_remaining;
	unsigned source_height_remaining, source_width_remaining;
	int hpos,vpos;
	uint16_t hsize,vsize;
	uint16_t palette;
	uint16_t linkno;
	uint16_t offset;
	uint16_t format;
	int tile_index;
	int num_cols,num_rows;
	int dx,dy;
	int row,col;
	int sx,sy,tile;
	int flipx,flipy;
	uint32_t zoomx, zoomy;
	int tile_screen_width;
	int tile_screen_height;
	const uint16_t *spriteformat16 = &spriteram16[0x4000/2];
	const uint16_t *spritetile16   = &spriteram16[0x8000/2];
	int color;
	const uint16_t *pWinAttr;
	rectangle clip;
	int xscroll, yscroll;

	/**
	 * ----xxxx-------- window select
	 * --------xxxx---- priority
	 * ------------xxxx palette select
	 */
	palette = pSource[6];
	if( pri != ((palette>>4)&0xf) )
	{
		return;
	}

	linkno      = pSource[0]; /* LINKNO */
	offset      = pSource[1]; /* OFFSET */
	hpos        = pSource[2]; /* HPOS       0x000..0x7ff (signed) */
	vpos        = pSource[3]; /* VPOS       0x000..0x7ff (signed) */
	hsize       = pSource[4]; /* HSIZE      max 0x3ff pixels */
	vsize       = pSource[5]; /* VSIZE      max 0x3ff pixels */
	/* pSource[6] contains priority/palette */
	/* pSource[7] is used in Lucky & Wild, possibly for sprite-road priority */

	if( linkno*4>=0x4000/2 ) return; /* avoid garbage memory reads */

	xscroll = (int16_t)m_position[1];
	yscroll = (int16_t)m_position[0];

//  xscroll &= 0x3ff; if( xscroll & 0x200 ) xscroll |= ~0x3ff;
	xscroll &= 0x1ff; if( xscroll & 0x100 ) xscroll |= ~0x1ff;
	yscroll &= 0x1ff; if( yscroll & 0x100 ) yscroll |= ~0x1ff;

	if( bitmap.width() > 384 )
	{ /* Medium Resolution: System21 adjust */
			xscroll = (int16_t)m_position[1];
			xscroll &= 0x3ff; if( xscroll & 0x200 ) xscroll |= ~0x3ff;
			if( yscroll<0 )
			{ /* solvalou */
				yscroll += 0x20;
			}
			yscroll += 0x10;
	}
	else
	{
		if (m_is_namcofl)
		{ /* Namco FL: don't adjust and things line up fine */
		}
		else
		{ /* Namco NB1, Namco System 2 */
			xscroll += 0x26;
			yscroll += 0x19;
		}
	}

	hpos -= xscroll;
	vpos -= yscroll;
	pWinAttr = &spriteram16[0x2400/2+((palette>>8)&0xf)*4];
	clip.set(pWinAttr[0] - xscroll, pWinAttr[1] - xscroll, pWinAttr[2] - yscroll, pWinAttr[3] - yscroll);
	clip &= cliprect;
	hpos&=0x7ff; if( hpos&0x400 ) hpos |= ~0x7ff; /* sign extend */
	vpos&=0x7ff; if( vpos&0x400 ) vpos |= ~0x7ff; /* sign extend */

	tile_index      = spriteformat16[linkno*4+0];
	format          = spriteformat16[linkno*4+1];
	dx              = spriteformat16[linkno*4+2];
	dy              = spriteformat16[linkno*4+3];
	num_cols        = (format>>4)&0xf;
	num_rows        = (format)&0xf;

	if( num_cols == 0 ) num_cols = 0x10;
	flipx = (hsize&0x8000)?1:0;
	hsize &= 0x3ff;//0x1ff;
	if( hsize == 0 ) return;
	zoomx = (hsize<<16)/(num_cols*16);
	dx = (dx*zoomx+0x8000)>>16;
	if( flipx )
	{
		hpos += dx;
	}
	else
	{
		hpos -= dx;
	}

	if( num_rows == 0 ) num_rows = 0x10;
	flipy = (vsize&0x8000)?1:0;
	vsize &= 0x3ff;
	if( vsize == 0 ) return;
	zoomy = (vsize<<16)/(num_rows*16);
	dy = (dy*zoomy+0x8000)>>16;
	if( flipy )
	{
		vpos += dy;
	}
	else
	{
		vpos -= dy;
	}

	color = (palette&0xf)^m_palxor;

	source_height_remaining = num_rows*16;
	screen_height_remaining = vsize;
	sy = vpos;
	for( row=0; row<num_rows; row++ )
	{
		tile_screen_height = 16*screen_height_remaining/source_height_remaining;
		zoomy = (screen_height_remaining<<16)/source_height_remaining;
		if( flipy )
		{
			sy -= tile_screen_height;
		}
		source_width_remaining = num_cols*16;
		screen_width_remaining = hsize;
		sx = hpos;
		for( col=0; col<num_cols; col++ )
		{
			tile_screen_width = 16*screen_width_remaining/source_width_remaining;
			zoomx = (screen_width_remaining<<16)/source_width_remaining;
			if( flipx )
			{
				sx -= tile_screen_width;
			}
			tile = spritetile16[tile_index++];
			if( (tile&0x8000)==0 )
			{
				zdrawgfxzoom(
					screen,
					bitmap,
					clip,
					m_gfxdecode->gfx(m_gfx_region),
					m_code2tile(tile) + offset,
					color,
					flipx,flipy,
					sx,sy,
					zoomx, zoomy, zpos );
			}
			if( !flipx )
			{
				sx += tile_screen_width;
			}
			screen_width_remaining -= tile_screen_width;
			source_width_remaining -= 16;
		} /* next col */
		if( !flipy )
		{
			sy += tile_screen_height;
		}
		screen_height_remaining -= tile_screen_height;
		source_height_remaining -= 16;
	} /* next row */
}


int namco_c355spr_device::default_code2tile(int code)
{
	return code;
}

template<class _BitmapClass>
void namco_c355spr_device::draw_list(screen_device &screen, _BitmapClass &bitmap, const rectangle &cliprect, int pri, const uint16_t *pSpriteList16, const uint16_t *pSpriteTable)
{
	int i;
	/* draw the sprites */
	for( i=0; i<256; i++ )
	{
		uint16_t which = pSpriteList16[i];
		draw_sprite(screen, bitmap, cliprect, &pSpriteTable[(which&0xff)*8], pri, i );
		if( which&0x100 ) break;
	}
}

void namco_c355spr_device::draw(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int pri)
{
//  int offs = spriteram16[0x18000/2]; /* end-of-sprite-list */
	if (pri == 0)
		screen.priority().fill(0, cliprect);

//  if (offs == 0)  // boot
	// TODO: solvalou service mode wants 0x14000/2 & 0x00000/2
		draw_list(screen, bitmap, cliprect, pri, &m_spriteram[0x02000/2], &m_spriteram[0x00000/2]);
//  else
		draw_list(screen, bitmap, cliprect, pri, &m_spriteram[0x14000/2], &m_spriteram[0x10000/2]);
}

void namco_c355spr_device::draw(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect, int pri)
{
//  int offs = spriteram16[0x18000/2]; /* end-of-sprite-list */
	if (pri == 0)
		screen.priority().fill(0, cliprect);

//  if (offs == 0)  // boot
		draw_list(screen, bitmap, cliprect, pri, &m_spriteram[0x02000/2], &m_spriteram[0x00000/2]);
//  else
		draw_list(screen, bitmap, cliprect, pri, &m_spriteram[0x14000/2], &m_spriteram[0x10000/2]);
}

WRITE16_MEMBER( namco_c355spr_device::spriteram_w )
{
	COMBINE_DATA(&m_spriteram[offset]);
}

READ16_MEMBER( namco_c355spr_device::spriteram_r )
{
	return m_spriteram[offset];
}

