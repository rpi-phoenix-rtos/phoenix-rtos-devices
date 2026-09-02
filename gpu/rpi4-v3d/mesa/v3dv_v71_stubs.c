/*
 * v3dv_v71_stubs.c — trap-stubs for the DEAD V3D-7.1 dispatch branch (Vulkan Tier 0).
 *
 * V3DV's v3d_X(devinfo, fn) macro (src/broadcom/common/v3d_util.h) is a runtime
 * switch on devinfo->ver that takes the ADDRESS of BOTH &v3d42_fn and &v3d71_fn, so
 * both symbol sets must resolve at link time even though only one branch ever runs.
 * On BCM2711 (Raspberry Pi 4) the V3D is generation 4.2, and v3dv_device.c hard-fails
 * ver < 42, so devinfo->ver == 42 EXACTLY and the case-71 branch is unreachable.
 *
 * We build the v3dvx_* front-end at V3D_VERSION=42 only (the v71 build would drag in a
 * whole V3D-7.1 backend closure — CLE/QPU at -DV3D_VERSION=71 — that libv3d-phoenix.a
 * does not contain, for a code path that can never execute on Pi4). Instead, satisfy
 * the linker with WEAK TRAP stubs: if the dead branch is ever somehow entered, abort
 * loudly rather than silently returning garbage (the silent-corruption failure class
 * this port keeps getting bitten by). 'weak' lets a real -DV3D_VERSION=71 backend
 * (e.g. a future Pi5 target) override these with zero edits here.
 *
 * Generated from the link-drive undefined-symbol list; see
 * docs/inprogress/2026-06-17-vulkan-v3dv-tier0-progress.md.
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#include <stdio.h>
#include <stdlib.h>

static void v3dv_v71_trap(const char *fn)
{
	fprintf(stderr, "v3dv: BUG: %s called — V3D-7.1 path on a V3D-4.2 device\n", fn);
	abort();
}

__attribute__((weak)) void v3d71_cmd_buffer_emit_blend(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_blend"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_color_write_mask(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_color_write_mask"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_configuration_bits(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_configuration_bits"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_default_point_size(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_default_point_size"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_depth_bias(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_depth_bias"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_depth_bounds(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_depth_bounds"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_draw(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_draw"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_draw_indexed(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_draw_indexed"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_draw_indirect(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_draw_indirect"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_gl_shader_state(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_gl_shader_state"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_index_buffer(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_index_buffer"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_indexed_indirect(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_indexed_indirect"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_line_width(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_line_width"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_occlusion_query(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_occlusion_query"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_render_pass_rcl(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_render_pass_rcl"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_sample_state(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_sample_state"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_stencil(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_stencil"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_varyings_state(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_varyings_state"); }
__attribute__((weak)) void v3d71_cmd_buffer_emit_viewport(void) { v3dv_v71_trap("v3d71_cmd_buffer_emit_viewport"); }
__attribute__((weak)) void v3d71_cmd_buffer_end_render_pass_secondary(void) { v3dv_v71_trap("v3d71_cmd_buffer_end_render_pass_secondary"); }
__attribute__((weak)) void v3d71_cmd_buffer_execute_inside_pass(void) { v3dv_v71_trap("v3d71_cmd_buffer_execute_inside_pass"); }
__attribute__((weak)) void v3d71_cmd_buffer_prepare_suspend_job_for_submit(void) { v3dv_v71_trap("v3d71_cmd_buffer_prepare_suspend_job_for_submit"); }
__attribute__((weak)) void v3d71_cmd_buffer_suspend(void) { v3dv_v71_trap("v3d71_cmd_buffer_suspend"); }
__attribute__((weak)) void v3d71_combined_image_sampler_sampler_state_offset(void) { v3dv_v71_trap("v3d71_combined_image_sampler_sampler_state_offset"); }
__attribute__((weak)) void v3d71_combined_image_sampler_texture_state_offset(void) { v3dv_v71_trap("v3d71_combined_image_sampler_texture_state_offset"); }
__attribute__((weak)) void v3d71_create_default_attribute_values(void) { v3dv_v71_trap("v3d71_create_default_attribute_values"); }
__attribute__((weak)) void v3d71_descriptor_bo_size(void) { v3dv_v71_trap("v3d71_descriptor_bo_size"); }
__attribute__((weak)) void v3d71_format_supports_blending(void) { v3dv_v71_trap("v3d71_format_supports_blending"); }
__attribute__((weak)) void v3d71_format_supports_tlb_resolve(void) { v3dv_v71_trap("v3d71_format_supports_tlb_resolve"); }
__attribute__((weak)) void v3d71_framebuffer_compute_internal_bpp_msaa(void) { v3dv_v71_trap("v3d71_framebuffer_compute_internal_bpp_msaa"); }
__attribute__((weak)) void v3d71_get_format(void) { v3dv_v71_trap("v3d71_get_format"); }
__attribute__((weak)) void v3d71_get_hw_clear_color(void) { v3dv_v71_trap("v3d71_get_hw_clear_color"); }
__attribute__((weak)) void v3d71_get_internal_depth_type(void) { v3dv_v71_trap("v3d71_get_internal_depth_type"); }
__attribute__((weak)) void v3d71_get_internal_type_bpp_for_image_aspects(void) { v3dv_v71_trap("v3d71_get_internal_type_bpp_for_image_aspects"); }
__attribute__((weak)) void v3d71_job_emit_binning_flush(void) { v3dv_v71_trap("v3d71_job_emit_binning_flush"); }
__attribute__((weak)) void v3d71_job_emit_binning_prolog(void) { v3dv_v71_trap("v3d71_job_emit_binning_prolog"); }
__attribute__((weak)) void v3d71_job_emit_clip_window(void) { v3dv_v71_trap("v3d71_job_emit_clip_window"); }
__attribute__((weak)) void v3d71_job_emit_noop(void) { v3dv_v71_trap("v3d71_job_emit_noop"); }
__attribute__((weak)) void v3d71_job_patch_resume_address(void) { v3dv_v71_trap("v3d71_job_patch_resume_address"); }
__attribute__((weak)) void v3d71_max_descriptor_bo_size(void) { v3dv_v71_trap("v3d71_max_descriptor_bo_size"); }
__attribute__((weak)) void v3d71_meta_copy_buffer(void) { v3dv_v71_trap("v3d71_meta_copy_buffer"); }
__attribute__((weak)) void v3d71_meta_emit_clear_image_rcl(void) { v3dv_v71_trap("v3d71_meta_emit_clear_image_rcl"); }
__attribute__((weak)) void v3d71_meta_emit_copy_buffer_to_image_rcl(void) { v3dv_v71_trap("v3d71_meta_emit_copy_buffer_to_image_rcl"); }
__attribute__((weak)) void v3d71_meta_emit_copy_image_rcl(void) { v3dv_v71_trap("v3d71_meta_emit_copy_image_rcl"); }
__attribute__((weak)) void v3d71_meta_emit_copy_image_to_buffer_rcl(void) { v3dv_v71_trap("v3d71_meta_emit_copy_image_to_buffer_rcl"); }
__attribute__((weak)) void v3d71_meta_emit_resolve_image_rcl(void) { v3dv_v71_trap("v3d71_meta_emit_resolve_image_rcl"); }
__attribute__((weak)) void v3d71_meta_emit_tfu_job(void) { v3dv_v71_trap("v3d71_meta_emit_tfu_job"); }
__attribute__((weak)) void v3d71_meta_fill_buffer(void) { v3dv_v71_trap("v3d71_meta_fill_buffer"); }
__attribute__((weak)) void v3d71_meta_framebuffer_init(void) { v3dv_v71_trap("v3d71_meta_framebuffer_init"); }
__attribute__((weak)) void v3d71_pack_null_texture_state(void) { v3dv_v71_trap("v3d71_pack_null_texture_state"); }
__attribute__((weak)) void v3d71_pack_sampler_state(void) { v3dv_v71_trap("v3d71_pack_sampler_state"); }
__attribute__((weak)) void v3d71_pack_texture_shader_state(void) { v3dv_v71_trap("v3d71_pack_texture_shader_state"); }
__attribute__((weak)) void v3d71_pack_texture_shader_state_from_buffer_view(void) { v3dv_v71_trap("v3d71_pack_texture_shader_state_from_buffer_view"); }
__attribute__((weak)) void v3d71_pipeline_needs_default_attribute_values(void) { v3dv_v71_trap("v3d71_pipeline_needs_default_attribute_values"); }
__attribute__((weak)) void v3d71_pipeline_pack_compile_state(void) { v3dv_v71_trap("v3d71_pipeline_pack_compile_state"); }
__attribute__((weak)) void v3d71_pipeline_pack_state(void) { v3dv_v71_trap("v3d71_pipeline_pack_state"); }
__attribute__((weak)) void v3d71_viewport_compute_xform(void) { v3dv_v71_trap("v3d71_viewport_compute_xform"); }
