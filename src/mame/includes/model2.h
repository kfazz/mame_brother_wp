// license:BSD-3-Clause
// copyright-holders:R. Belmont, Olivier Galibert, ElSemi, Angelo Salese
#include "video/poly.h"
#include "audio/dsbz80.h"
#include "audio/segam1audio.h"
#include "machine/eepromser.h"
#include "machine/i8251.h"
#include "cpu/i960/i960.h"
#include "cpu/mb86235/mb86235.h"
#include "sound/scsp.h"
#include "machine/315-5881_crypt.h"
#include "machine/315-5838_317-0229_comp.h"
#include "machine/m2comm.h"
#include "machine/timer.h"
#include "screen.h"

class model2_renderer;
struct raster_state;
struct geo_state;
struct triangle;

class model2_state : public driver_device
{
public:
	model2_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_workram(*this, "workram"),
		m_bufferram(*this, "bufferram"),
		m_textureram0(*this, "textureram0"),
		m_textureram1(*this, "textureram1"),
		m_soundram(*this, "soundram"),
		m_tgp_program(*this, "tgp_program"),
		m_tgpx4_program(*this, "tgpx4_program"),
		m_maincpu(*this,"maincpu"),
		m_dsbz80(*this, DSBZ80_TAG),
		m_m1audio(*this, M1AUDIO_TAG),
		m_uart(*this, "uart"),
		m_m2comm(*this, "m2comm"),
		m_audiocpu(*this, "audiocpu"),
		m_tgp(*this, "tgp"),
		m_dsp(*this, "dsp"),
		m_tgpx4(*this, "tgpx4"),
		m_drivecpu(*this, "drivecpu"),
		m_eeprom(*this, "eeprom"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_scsp(*this, "scsp"),
		m_cryptdevice(*this, "315_5881"),
		m_0229crypt(*this, "317_0229"),
		m_in(*this, "IN%u", 0),
		m_dsw(*this, "DSW"),
		m_steer(*this, "STEER"),
		m_accel(*this, "ACCEL"),
		m_brake(*this, "BRAKE"),
		m_gears(*this, "GEARS"),
		m_analog_ports(*this, "ANA%u", 0),
		m_lightgun_ports(*this, {"P1_Y", "P1_X", "P2_Y", "P2_X"})
	{ }

	required_shared_ptr<uint32_t> m_workram;
	required_shared_ptr<uint32_t> m_bufferram;
	std::unique_ptr<uint16_t[]> m_palram;
	std::unique_ptr<uint16_t[]> m_colorxlat;
	required_shared_ptr<uint32_t> m_textureram0;
	required_shared_ptr<uint32_t> m_textureram1;
	std::unique_ptr<uint16_t[]> m_lumaram;
	std::unique_ptr<uint16_t[]> m_fbvramA;
	std::unique_ptr<uint16_t[]> m_fbvramB;
	optional_shared_ptr<uint16_t> m_soundram;
	optional_shared_ptr<uint32_t> m_tgp_program;
	optional_shared_ptr<uint64_t> m_tgpx4_program;

	required_device<i960_cpu_device> m_maincpu;
	optional_device<dsbz80_device> m_dsbz80;    // Z80-based MPEG Digital Sound Board
	optional_device<segam1audio_device> m_m1audio;  // Model 1 standard sound board
	required_device<i8251_device> m_uart;
	optional_device<m2comm_device> m_m2comm;        // Model 2 communication board
	optional_device<cpu_device> m_audiocpu;
	optional_device<cpu_device> m_tgp;
	optional_device<cpu_device> m_dsp;
	optional_device<mb86235_device> m_tgpx4;
	optional_device<cpu_device> m_drivecpu;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	optional_device<scsp_device> m_scsp;
	optional_device<sega_315_5881_crypt_device> m_cryptdevice;
	optional_device<sega_315_5838_comp_device> m_0229crypt;

	optional_ioport_array<5> m_in;
	required_ioport m_dsw;
	optional_ioport m_steer;
	optional_ioport m_accel;
	optional_ioport m_brake;
	optional_ioport m_gears;
	optional_ioport_array<4> m_analog_ports;
	optional_ioport_array<4> m_lightgun_ports;

	uint32_t m_timervals[4];
	uint32_t m_timerorig[4];
	int m_timerrun[4];
	timer_device *m_timers[4];
	int m_ctrlmode;
	int m_analog_channel;
	int m_dsp_type;
	int m_copro_fifoin_rpos;
	int m_copro_fifoin_wpos;
	std::unique_ptr<uint32_t[]> m_copro_fifoin_data;
	int m_copro_fifoin_num;
	int m_copro_fifoout_rpos;
	int m_copro_fifoout_wpos;
	std::unique_ptr<uint32_t[]> m_copro_fifoout_data;
	int m_copro_fifoout_num;
	uint16_t m_cmd_data;
	uint8_t m_driveio_comm_data;
	int m_iop_write_num;
	uint32_t m_iop_data;
	int m_geo_iop_write_num;
	uint32_t m_geo_iop_data;

	uint32_t m_geo_read_start_address;
	uint32_t m_geo_write_start_address;
	model2_renderer *m_poly;
	raster_state *m_raster;
	geo_state *m_geo;
	bitmap_rgb32 m_sys24_bitmap;
//  uint32_t m_soundack;
	void model2_check_irq_state();
	void model2_check_irqack_state(uint32_t data);
	uint8_t m_gearsel;
	uint8_t m_lightgun_mux;

	DECLARE_READ8_MEMBER(model2_crx_in_r);
	DECLARE_CUSTOM_INPUT_MEMBER(daytona_gearbox_r);
	DECLARE_CUSTOM_INPUT_MEMBER(rchase2_devices_r);
	DECLARE_READ32_MEMBER(timers_r);
	DECLARE_WRITE32_MEMBER(timers_w);
	DECLARE_READ16_MEMBER(palette_r);
	DECLARE_WRITE16_MEMBER(palette_w);
	DECLARE_READ16_MEMBER(colorxlat_r);
	DECLARE_WRITE16_MEMBER(colorxlat_w);
	DECLARE_WRITE32_MEMBER(ctrl0_w);
	DECLARE_WRITE32_MEMBER(analog_2b_w);
	DECLARE_READ32_MEMBER(fifo_control_2a_r);
	DECLARE_READ32_MEMBER(videoctl_r);
	DECLARE_WRITE32_MEMBER(videoctl_w);
	DECLARE_WRITE32_MEMBER(rchase2_devices_w);
	DECLARE_WRITE32_MEMBER(srallyc_devices_w);
	DECLARE_READ32_MEMBER(copro_prg_r);
	DECLARE_WRITE32_MEMBER(copro_prg_w);
	DECLARE_READ32_MEMBER(copro_ctl1_r);
	DECLARE_WRITE32_MEMBER(copro_ctl1_w);
	DECLARE_WRITE32_MEMBER(copro_function_port_w);
	DECLARE_READ32_MEMBER(copro_fifo_r);
	DECLARE_WRITE32_MEMBER(copro_fifo_w);
	DECLARE_WRITE32_MEMBER(copro_sharc_iop_w);
	DECLARE_WRITE32_MEMBER(geo_ctl1_w);
	DECLARE_READ32_MEMBER(geo_prg_r);
	DECLARE_WRITE32_MEMBER(geo_prg_w);
	DECLARE_READ32_MEMBER(geo_r);
	DECLARE_WRITE32_MEMBER(geo_w);
	DECLARE_READ8_MEMBER(hotd_lightgun_r);
	DECLARE_WRITE32_MEMBER(hotd_lightgun_w);
	DECLARE_READ32_MEMBER(irq_request_r);
	DECLARE_WRITE32_MEMBER(irq_ack_w);
	DECLARE_READ32_MEMBER(irq_enable_r);
	DECLARE_WRITE32_MEMBER(irq_enable_w);
	DECLARE_READ32_MEMBER(model2_serial_r);
	DECLARE_WRITE32_MEMBER(model2o_serial_w);
	DECLARE_WRITE32_MEMBER(model2_serial_w);
	DECLARE_WRITE16_MEMBER(horizontal_sync_w);
	DECLARE_WRITE16_MEMBER(vertical_sync_w);

	void raster_init(memory_region *texture_rom);
	void geo_init(memory_region *polygon_rom);
	DECLARE_READ32_MEMBER(render_mode_r);
	DECLARE_WRITE32_MEMBER(render_mode_w);
	DECLARE_WRITE32_MEMBER(model2o_tex_w0);
	DECLARE_WRITE32_MEMBER(model2o_tex_w1);
	DECLARE_READ16_MEMBER(lumaram_r);
	DECLARE_WRITE16_MEMBER(lumaram_w);
	DECLARE_READ16_MEMBER(fbvram_bankA_r);
	DECLARE_WRITE16_MEMBER(fbvram_bankA_w);
	DECLARE_READ16_MEMBER(fbvram_bankB_r);
	DECLARE_WRITE16_MEMBER(fbvram_bankB_w);
	DECLARE_WRITE32_MEMBER(model2_3d_zclip_w);
	DECLARE_WRITE16_MEMBER(model2snd_ctrl);
	DECLARE_READ32_MEMBER(copro_sharc_input_fifo_r);
	DECLARE_WRITE32_MEMBER(copro_sharc_output_fifo_w);
	DECLARE_READ32_MEMBER(copro_sharc_buffer_r);
	DECLARE_WRITE32_MEMBER(copro_sharc_buffer_w);
	DECLARE_READ32_MEMBER(copro_tgp_buffer_r);
	DECLARE_WRITE32_MEMBER(copro_tgp_buffer_w);
	DECLARE_READ8_MEMBER(tgpid_r);
	DECLARE_READ32_MEMBER(copro_status_r);
	DECLARE_READ32_MEMBER(polygon_count_r);

	DECLARE_READ8_MEMBER(driveio_portg_r);
	DECLARE_READ8_MEMBER(driveio_porth_r);
	DECLARE_WRITE8_MEMBER(driveio_port_w);
	void push_geo_data(uint32_t data);
	DECLARE_DRIVER_INIT(overrev);
	DECLARE_DRIVER_INIT(pltkids);
	DECLARE_DRIVER_INIT(rchase2);
	DECLARE_DRIVER_INIT(manxttdx);
	DECLARE_DRIVER_INIT(srallyc);
	DECLARE_DRIVER_INIT(doa);
	DECLARE_DRIVER_INIT(zerogun);
	DECLARE_DRIVER_INIT(sgt24h);
	DECLARE_MACHINE_START(model2);
	DECLARE_MACHINE_START(srallyc);
	DECLARE_MACHINE_RESET(model2o);
	DECLARE_VIDEO_START(model2);
	DECLARE_MACHINE_RESET(model2);
	DECLARE_MACHINE_RESET(model2b);
	DECLARE_MACHINE_RESET(model2c);
	DECLARE_MACHINE_RESET(model2_common);
	DECLARE_MACHINE_RESET(model2_scsp);
	uint32_t screen_update_model2(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
//  DECLARE_WRITE_LINE_MEMBER(screen_vblank_model2);
//  DECLARE_WRITE_LINE_MEMBER(sound_ready_w);
	TIMER_DEVICE_CALLBACK_MEMBER(model2_timer_cb);
	TIMER_DEVICE_CALLBACK_MEMBER(model2_interrupt);
	TIMER_DEVICE_CALLBACK_MEMBER(model2c_interrupt);
	DECLARE_WRITE8_MEMBER(scsp_irq);
	DECLARE_READ_LINE_MEMBER(copro_tgp_fifoin_pop_ok);
	DECLARE_READ32_MEMBER(copro_tgp_fifoin_pop);
	DECLARE_WRITE32_MEMBER(copro_tgp_fifoout_push);
	DECLARE_READ8_MEMBER(virtuacop_lightgun_r);
	DECLARE_READ8_MEMBER(virtuacop_lightgun_offscreen_r);

	uint16_t crypt_read_callback(uint32_t addr);

	bool copro_fifoin_pop(device_t *device, uint32_t *result,uint32_t offset, uint32_t mem_mask);
	void copro_fifoin_push(device_t *device, uint32_t data, uint32_t offset, uint32_t mem_mask);
	uint32_t copro_fifoout_pop(address_space &space, uint32_t offset, uint32_t mem_mask);
	void copro_fifoout_push(device_t *device, uint32_t data,uint32_t offset,uint32_t mem_mask);

	void model2_3d_frame_start( void );
	void geo_parse( void );
	void model2_3d_frame_end( bitmap_rgb32 &bitmap, const rectangle &cliprect );
	void draw_framebuffer(bitmap_rgb32 &bitmap, const rectangle &cliprect );

	void model2_timers(machine_config &config);
	void model2_screen(machine_config &config);
	void model2_scsp(machine_config &config);

	void sj25_0207_01(machine_config &config);
	void copro_sharc_map(address_map &map);
	void copro_tgp_map(address_map &map);
	void drive_io_map(address_map &map);
	void drive_map(address_map &map);
	void geo_sharc_map(address_map &map);
	void model2_base_mem(address_map &map);
	void model2_5881_mem(address_map &map);
	void model2_snd(address_map &map);

	uint8_t m_gamma_table[256];

	void debug_init();
	void debug_commands( int ref, const std::vector<std::string> &params );
	void debug_geo_dasm_command(int ref, const std::vector<std::string> &params);
	void debug_tri_dump_command(int ref, const std::vector<std::string> &params);
	void debug_help_command(int ref, const std::vector<std::string> &params);

protected:
	virtual void video_start() override;

private:
	void tri_list_dump(FILE *dst);

	uint32_t m_intreq;
	uint32_t m_intena;
	uint32_t m_coproctl;
	uint32_t m_coprocnt;
	uint32_t m_geoctl;
	uint32_t m_geocnt;
	uint32_t m_videocontrol;
	int m_port_1c00004;
	int m_port_1c00006;
	int m_port_1c00010;
	int m_port_1c00012;
	int m_port_1c00014;

	bool m_render_unk;
	bool m_render_mode;
	bool m_render_test_mode;
	int16 m_crtc_xoffset, m_crtc_yoffset;

	uint32_t *geo_process_command( geo_state *geo, uint32_t opcode, uint32_t *input, bool *end_code );
	// geo commands
	uint32_t *geo_nop( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_object_data( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_direct_data( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_window_data( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_texture_data( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_polygon_data( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_texture_parameters( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_mode( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_zsort_mode( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_focal_distance( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_light_source( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_matrix_write( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_translate_write( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_data_mem_push( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_test( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_end( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_dummy( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_log_data( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_lod( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_code_upload( geo_state *geo, uint32_t opcode, uint32_t *input );
	uint32_t *geo_code_jump( geo_state *geo, uint32_t opcode, uint32_t *input );
	// geo code drawing paths
	void geo_parse_np_ns( geo_state *geo, uint32_t *input, uint32_t count );
	void geo_parse_np_s( geo_state *geo, uint32_t *input, uint32_t count );
	void geo_parse_nn_ns( geo_state *geo, uint32_t *input, uint32_t count );
	void geo_parse_nn_s( geo_state *geo, uint32_t *input, uint32_t count );

	// raster functions
	// main data input port
	void model2_3d_push( raster_state *raster, uint32_t input );
	// quad & triangle push paths
	void model2_3d_process_quad( raster_state *raster, uint32_t attr );
	void model2_3d_process_triangle( raster_state *raster, uint32_t attr );

	// inliners
	inline void model2_3d_project( triangle *tri );
	inline uint16_t float_to_zval( float floatval );
	inline bool check_culling( raster_state *raster, uint32_t attr, float min_z, float max_z );
};

/*****************************
 *
 * Model 2
 *
 *****************************/

class model2o_state : public model2_state
{
public:
	model2o_state(const machine_config &mconfig, device_type type, const char *tag)
		: model2_state(mconfig, type, tag)
	{}

	DECLARE_READ32_MEMBER(daytona_unk_r);
	DECLARE_READ8_MEMBER(model2o_in_r);
	DECLARE_READ32_MEMBER(fifo_control_2o_r);

	void daytona(machine_config &config);
	void model2o(machine_config &config);
	void model2o_mem(address_map &map);
};

/*****************************
 *
 * Daytona To The Maxx
 *
 *****************************/

class model2o_maxx_state : public model2o_state
{
public:
	model2o_maxx_state(const machine_config &mconfig, device_type type, const char *tag)
		: model2o_state(mconfig, type, tag)
	{}

	DECLARE_READ32_MEMBER(maxx_r);
	void daytona_maxx(machine_config &config);
	void model2o_maxx_mem(address_map &map);

private:
	int m_maxxstate;
};

/*****************************
 *
 * Daytona GTX 2004 Edition
 *
 *****************************/

class model2o_gtx_state : public model2o_state
{
public:
	model2o_gtx_state(const machine_config &mconfig, device_type type, const char *tag)
		: model2o_state(mconfig, type, tag)
	{}

	DECLARE_READ8_MEMBER(gtx_r);
	void daytona_gtx(machine_config &config);
	void model2o_gtx_mem(address_map &map);

private:
	int m_gtx_state;
};

/*****************************
 *
 * Model 2A
 *
 *****************************/

class model2a_state : public model2_state
{
public:
	model2a_state(const machine_config &mconfig, device_type type, const char *tag)
		: model2_state(mconfig, type, tag)
	{}

	void manxtt(machine_config &config);
	void manxttdx(machine_config &config);
	void model2a(machine_config &config);
	void model2a_0229(machine_config &config);
	void model2a_5881(machine_config &config);
	void srallyc(machine_config &config);
	void model2a_crx_mem(address_map &map);
	void model2a_5881_mem(address_map &map);
};

/*****************************
 *
 * Model 2B
 *
 *****************************/

class model2b_state : public model2_state
{
public:
	model2b_state(const machine_config &mconfig, device_type type, const char *tag)
		: model2_state(mconfig, type, tag)
	{}

	void model2b(machine_config &config);
	void model2b_0229(machine_config &config);
	void model2b_5881(machine_config &config);
	void indy500(machine_config &config);
	void rchase2(machine_config &config);
	void model2b_crx_mem(address_map &map);
	void model2b_5881_mem(address_map &map);
	// TODO: split into own class
	void rchase2_iocpu_map(address_map &map);
	void rchase2_ioport_map(address_map &map);
};

/*****************************
 *
 * Model 2C
 *
 *****************************/

class model2c_state : public model2_state
{
public:
	model2c_state(const machine_config &mconfig, device_type type, const char *tag)
		: model2_state(mconfig, type, tag)
	{}

	DECLARE_READ32_MEMBER(fifo_control_2c_r);

	void model2c(machine_config &config);
	void model2c_5881(machine_config &config);
	void overrev2c(machine_config &config);
	void stcc(machine_config &config);
	void model2c_crx_mem(address_map &map);
	void model2c_5881_mem(address_map &map);
	void copro_tgpx4_map(address_map &map);
	void copro_tgpx4_data_map(address_map &map);
};

/*****************************
 *
 * Modern polygon renderer
 *
 *****************************/

struct m2_poly_extra_data
{
	model2_state *  state;
	uint32_t      lumabase;
	uint32_t      colorbase;
	uint32_t *    texsheet;
	uint32_t      texwidth;
	uint32_t      texheight;
	uint32_t      texx, texy;
	uint8_t       texmirrorx;
	uint8_t       texmirrory;
};


static inline uint16_t get_texel( uint32_t base_x, uint32_t base_y, int x, int y, uint32_t *sheet )
{
	uint32_t  baseoffs = ((base_y/2)*512)+(base_x/2);
	uint32_t  texeloffs = ((y/2)*512)+(x/2);
	uint32_t  offset = baseoffs + texeloffs;
	uint32_t  texel = sheet[offset>>1];

	if ( offset & 1 )
		texel >>= 16;

	if ( (y & 1) == 0 )
		texel >>= 8;

	if ( (x & 1) == 0 )
		texel >>= 4;

	return (texel & 0x0f);
}

// 0x10000 = size of the tri_sorted_list array
class model2_renderer : public poly_manager<float, m2_poly_extra_data, 4, 0x10000>
{
public:
	typedef void (model2_renderer::*scanline_render_func)(int32_t scanline, const extent_t& extent, const m2_poly_extra_data& object, int threadid);

public:
	model2_renderer(model2_state& state)
		: poly_manager<float, m2_poly_extra_data, 4, 0x10000>(state.machine())
		, m_state(state)
		, m_destmap(512, 512)
	{
		m_renderfuncs[0] = &model2_renderer::model2_3d_render_0;
		m_renderfuncs[1] = &model2_renderer::model2_3d_render_1;
		m_renderfuncs[2] = &model2_renderer::model2_3d_render_2;
		m_renderfuncs[3] = &model2_renderer::model2_3d_render_3;
		m_renderfuncs[4] = &model2_renderer::model2_3d_render_4;
		m_renderfuncs[5] = &model2_renderer::model2_3d_render_5;
		m_renderfuncs[6] = &model2_renderer::model2_3d_render_6;
		m_renderfuncs[7] = &model2_renderer::model2_3d_render_7;
		m_xoffs = 90;
		m_yoffs = -8;
	}

	bitmap_rgb32& destmap() { return m_destmap; }

	void model2_3d_render(triangle *tri, const rectangle &cliprect);
	void set_xoffset(int16 xoffs) { m_xoffs = xoffs; }
	void set_yoffset(int16 yoffs) { m_yoffs = yoffs; }

	/* checker = 0, textured = 0, transparent = 0 */
	#define MODEL2_FUNC 0
	#define MODEL2_FUNC_NAME    model2_3d_render_0
	#include "video/model2rd.hxx"
	#undef MODEL2_FUNC
	#undef MODEL2_FUNC_NAME

	/* checker = 0, textured = 0, translucent = 1 */
	#define MODEL2_FUNC 1
	#define MODEL2_FUNC_NAME    model2_3d_render_1
	#include "video/model2rd.hxx"
	#undef MODEL2_FUNC
	#undef MODEL2_FUNC_NAME

	/* checker = 0, textured = 1, translucent = 0 */
	#define MODEL2_FUNC 2
	#define MODEL2_FUNC_NAME    model2_3d_render_2
	#include "video/model2rd.hxx"
	#undef MODEL2_FUNC
	#undef MODEL2_FUNC_NAME

	/* checker = 0, textured = 1, translucent = 1 */
	#define MODEL2_FUNC 3
	#define MODEL2_FUNC_NAME    model2_3d_render_3
	#include "video/model2rd.hxx"
	#undef MODEL2_FUNC
	#undef MODEL2_FUNC_NAME

	/* checker = 1, textured = 0, translucent = 0 */
	#define MODEL2_FUNC 4
	#define MODEL2_FUNC_NAME    model2_3d_render_4
	#include "video/model2rd.hxx"
	#undef MODEL2_FUNC
	#undef MODEL2_FUNC_NAME

	/* checker = 1, textured = 0, translucent = 1 */
	#define MODEL2_FUNC 5
	#define MODEL2_FUNC_NAME    model2_3d_render_5
	#include "video/model2rd.hxx"
	#undef MODEL2_FUNC
	#undef MODEL2_FUNC_NAME

	/* checker = 1, textured = 1, translucent = 0 */
	#define MODEL2_FUNC 6
	#define MODEL2_FUNC_NAME    model2_3d_render_6
	#include "video/model2rd.hxx"
	#undef MODEL2_FUNC
	#undef MODEL2_FUNC_NAME

	/* checker = 1, textured = 1, translucent = 1 */
	#define MODEL2_FUNC 7
	#define MODEL2_FUNC_NAME    model2_3d_render_7
	#include "video/model2rd.hxx"
	#undef MODEL2_FUNC
	#undef MODEL2_FUNC_NAME

	scanline_render_func m_renderfuncs[8];

private:
	model2_state& m_state;
	bitmap_rgb32 m_destmap;
	int16_t m_xoffs,m_yoffs;
};

typedef model2_renderer::vertex_t poly_vertex;


/*******************************************
 *
 *  Basic Data Types
 *
 *******************************************/

struct plane
{
	poly_vertex normal;
	float       distance;
};

struct texture_parameter
{
	float   diffuse;
	float   ambient;
	uint32_t  specular_control;
	float   specular_scale;
};

struct triangle
{
	void *              next;
	poly_vertex         v[3];
	uint16_t              z;
	uint16_t              texheader[4];
	uint8_t               luma;
	int16_t               viewport[4];
	int16_t               center[2];
};

struct quad_m2
{
	poly_vertex         v[4];
	uint16_t              z;
	uint16_t              texheader[4];
	uint8_t               luma;
};
