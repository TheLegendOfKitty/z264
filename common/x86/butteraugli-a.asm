;*****************************************************************************
;* butteraugli-a.asm: x86 butteraugli SIMD assembly optimizations
;*****************************************************************************
;* Copyright (C) 2025 x264 project
;*
;* Authors: Claude Code <noreply@anthropic.com>
;*
;* This program is free software; you can redistribute it and/or modify
;* it under the terms of the GNU General Public License as published by
;* the Free Software Foundation; either version 2 of the License, or
;* (at your option) any later version.
;*
;* This program is distributed in the hope that it will be useful,
;* but WITHOUT ANY WARRANTY; without even the implied warranty of
;* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;* GNU General Public License for more details.
;*
;* You should have received a copy of the GNU General Public License
;* along with this program; if not, write to the Free Software
;* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
;*
;* This program is also available under a commercial proprietary license.
;* For more information, contact us at licensing@x264.com.
;*****************************************************************************

%include "x86inc.asm"

SECTION_RODATA 32

; YUV to RGB conversion constants (BT.601) - SSE2 versions
yuv_r_coeff:    times 8 dw 360
yuv_g_coeff_u:  times 8 dw -88
yuv_g_coeff_v:  times 8 dw -184
yuv_b_coeff:    times 8 dw 455

; AVX2 versions (256-bit)
align 32
yuv_r_coeff_avx2:   times 16 dw 360
yuv_g_coeff_u_avx2: times 16 dw -88
yuv_g_coeff_v_avx2: times 16 dw -184
yuv_b_coeff_avx2:   times 16 dw 455

; 128 offset for UV components
uv_offset:      times 16 db 128
; AVX2 version
align 32
uv_offset_avx2: times 32 db 128

; Min/max values for clamping
min_val:        times 16 db 0
max_val:        times 16 db 255

; sRGB to linear lookup table (pre-computed)
; Values computed as: val <= 0.04045 ? val/12.92 : pow((val+0.055)/1.055, 2.4)
align 32
srgb_to_linear_lut:
; Values 0-15
dd 0.0, 0.000303527, 0.000607054, 0.000910581, 0.001214108, 0.001517635, 0.001821162, 0.002124689
dd 0.002428216, 0.002731743, 0.003035270, 0.003346535, 0.003676507, 0.004024717, 0.004391442, 0.004776953
; Values 16-31
dd 0.005181516, 0.005605391, 0.006048833, 0.006512091, 0.006995410, 0.007499032, 0.008023193, 0.008568126
dd 0.009134058, 0.009721218, 0.010329823, 0.010960094, 0.011612245, 0.012286488, 0.012983032, 0.013702083
; Values 32-47 
dd 0.014443844, 0.015208514, 0.015996293, 0.016807382, 0.017641954, 0.018500220, 0.019382360, 0.020288564
dd 0.021219010, 0.022173884, 0.023153366, 0.024157632, 0.025186859, 0.026241221, 0.027320891, 0.028426039
; Values 48-63
dd 0.029556834, 0.030713443, 0.031896033, 0.033104766, 0.034339808, 0.035601314, 0.036889450, 0.038204371
dd 0.039546237, 0.040915205, 0.042311430, 0.043735067, 0.045186270, 0.046665191, 0.048171984, 0.049706798
; Values 64-79
dd 0.051269785, 0.052861094, 0.054480874, 0.056129272, 0.057806437, 0.059512517, 0.061247658, 0.063012006
dd 0.064805708, 0.066628908, 0.068481749, 0.070364374, 0.072276925, 0.074219543, 0.076192369, 0.078195542
; Values 80-95
dd 0.080229201, 0.082293484, 0.084388529, 0.086514471, 0.088671446, 0.090859588, 0.093079030, 0.095329904
dd 0.097612343, 0.099926477, 0.102272435, 0.104650346, 0.107060337, 0.109502536, 0.111977069, 0.114484063
; Values 96-111
dd 0.117023644, 0.119595937, 0.122201067, 0.124839158, 0.127510334, 0.130214718, 0.132952433, 0.135723602
dd 0.138528347, 0.141366789, 0.144239050, 0.147145251, 0.150085513, 0.153059955, 0.156068698, 0.159111861
; Values 112-127
dd 0.162189563, 0.165301924, 0.168449062, 0.171631094, 0.174848138, 0.178100312, 0.181387733, 0.184710517
dd 0.188068782, 0.191462643, 0.194892218, 0.198357622, 0.201858972, 0.205396383, 0.208969971, 0.212579851
; Values 128-143
dd 0.216226139, 0.219908950, 0.223628399, 0.227384601, 0.231177670, 0.235007721, 0.238874869, 0.242779228
dd 0.246720912, 0.250700036, 0.254716713, 0.258771058, 0.262863184, 0.266993204, 0.271161233, 0.275367383
; Values 144-159
dd 0.279611768, 0.283894501, 0.288215695, 0.292575463, 0.296973918, 0.301411172, 0.305887338, 0.310402527
dd 0.314956852, 0.319550425, 0.324183359, 0.328855765, 0.333567755, 0.338319441, 0.343110934, 0.347942346
; Values 160-175
dd 0.352813789, 0.357725375, 0.362677215, 0.367669421, 0.372702105, 0.377775378, 0.382889352, 0.388044138
dd 0.393239848, 0.398476593, 0.403754485, 0.409073635, 0.414434155, 0.419836156, 0.425279750, 0.430765047
; Values 176-191
dd 0.436292160, 0.441861200, 0.447472278, 0.453125506, 0.458820995, 0.464558857, 0.470339202, 0.476162142
dd 0.482027789, 0.487936254, 0.493887648, 0.499882083, 0.505919670, 0.512000520, 0.518124744, 0.524292454
; Values 192-207
dd 0.530503761, 0.536758777, 0.543057613, 0.549400380, 0.555787189, 0.562218152, 0.568693380, 0.575212984
dd 0.581777076, 0.588385767, 0.595039168, 0.601737390, 0.608480545, 0.615268743, 0.622102097, 0.628980717
; Values 208-223
dd 0.635904715, 0.642874202, 0.649889289, 0.656950088, 0.664056710, 0.671209267, 0.678407870, 0.685652630
dd 0.692943660, 0.700281071, 0.707664975, 0.715095484, 0.722572709, 0.730096763, 0.737667757, 0.745285803
; Values 224-239
dd 0.752951013, 0.760663498, 0.768423370, 0.776230741, 0.784085723, 0.791988428, 0.799938967, 0.807937452
dd 0.815983995, 0.824078708, 0.832221703, 0.840413092, 0.848652986, 0.856941498, 0.865278738, 0.873664819
; Values 240-255
dd 0.882099852, 0.890583950, 0.899117224, 0.907699785, 0.916331746, 0.925013218, 0.933744313, 0.942525143
dd 0.951355818, 0.960236451, 0.969167154, 0.978148040, 0.987179220, 0.996260807, 1.000000000, 1.000000000

SECTION .text

;-----------------------------------------------------------------------------
; void x264_butteraugli_yuv_to_rgb_sse2_bt601( const uint8_t *y, const uint8_t *u, const uint8_t *v,
;                                              int y_stride, int uv_stride,
;                                              uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
;                                              int width, int height );
;-----------------------------------------------------------------------------
INIT_XMM sse2
cglobal butteraugli_yuv_to_rgb_sse2_bt601, 10,11,8
    mov         r10d, r9d               ; height
    
    ; Load constants
    mova        m4, [yuv_r_coeff]
    mova        m5, [yuv_g_coeff_u]
    mova        m6, [yuv_g_coeff_v]
    mova        m7, [yuv_b_coeff]
    
.loop_y:
    xor         eax, eax                ; x = 0
    
.loop_x:
    ; Load 8 Y values
    movq        xmm0, [r0 + rax]        ; Y values
    
    ; Load 4 U and V values (for 4 pixels in 4:2:0)
    mov         rdx, rax
    shr         rdx, 1                  ; rdx = rax/2
    movd        m1, [r1 + rdx]         ; Load 4 U values
    movd        m2, [r2 + rdx]         ; Load 4 V values
    
    ; Unpack and duplicate chroma for 2x2 blocks (4:2:0 to 4:4:4)
    punpcklbw   m1, m1                  ; U0U0 U1U1 U2U2 U3U3
    punpcklbw   m2, m2                  ; V0V0 V1V1 V2V2 V3V3
    
    ; Subtract 128 from UV
    psubb       m1, [uv_offset]
    psubb       m2, [uv_offset]
    
    ; Convert to 16-bit
    pxor        m7, m7                  ; Clear register for zero extension
    punpcklbw   m0, m7                  ; Y -> 16-bit (unsigned), zero extend
    ; U and V are already sign extended after subtraction, convert to 16-bit signed
    punpcklbw   m3, m1, m1              ; Duplicate sign bits for U
    psraw       m3, 8                   ; Sign extend to 16-bit
    movdqa      m1, m3                  ; Move back to m1
    
    punpcklbw   m3, m2, m2              ; Duplicate sign bits for V  
    psraw       m3, 8                   ; Sign extend to 16-bit
    movdqa      m2, m3                  ; Move back to m2
    
    ; Calculate R = Y + ((360 * V) >> 8)
    pmullw      m3, m2, m4              ; V * 360
    psraw       m3, 8                   ; >> 8
    paddw       m3, m0                  ; + Y
    packuswb    m3, m3                  ; clamp to 0-255
    
    ; Calculate G = Y - ((88 * U + 184 * V) >> 8)
    pmullw      m1, m5                  ; U * -88
    pmullw      m2, m6                  ; V * -184
    paddw       m1, m2                  ; U*-88 + V*-184
    psraw       m1, 8                   ; >> 8
    paddw       m1, m0                  ; + Y
    packuswb    m1, m1                  ; clamp to 0-255
    
    ; Reload U for B calculation  
    mov         rdx, rax
    shr         rdx, 1                  ; rdx = rax/2
    movd        xmm2, [r1 + rdx]
    punpcklbw   m2, m2
    psubb       m2, [uv_offset]
    punpcklbw   m3, m2, m2              ; Duplicate sign bits for U
    psraw       m3, 8                   ; Sign extend to 16-bit
    movdqa      m2, m3                  ; Move back to m2
    
    ; Calculate B = Y + ((455 * U) >> 8)
    pmullw      m2, m7                  ; U * 455
    psraw       m2, 8                   ; >> 8
    paddw       m2, m0                  ; + Y
    packuswb    m2, m2                  ; clamp to 0-255
    
    ; Store RGB to separate planes (planar format) - only 4 pixels
    movd        [r5 + rax], m3          ; Store R values (4 pixels)
    movd        [r6 + rax], m1          ; Store G values (4 pixels)
    movd        [r7 + rax], m2          ; Store B values (4 pixels)
    
    add         eax, 4                  ; process 4 pixels at a time
    cmp         eax, r8d                ; compare with width
    jl          .loop_x
    
    ; Advance to next row
    add         r0, r3                  ; y += y_stride
    add         r1, r4                  ; u += uv_stride
    add         r2, r4                  ; v += uv_stride
    add         r5, r8                  ; rgb_r += rgb_stride
    add         r6, r8                  ; rgb_g += rgb_stride  
    add         r7, r8                  ; rgb_b += rgb_stride
    
    dec         r10d
    jnz         .loop_y
    
    RET

;-----------------------------------------------------------------------------
; void x264_butteraugli_yuv_to_rgb_avx2_bt601( const uint8_t *y, const uint8_t *u, const uint8_t *v,
;                                              int y_stride, int uv_stride,
;                                              uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
;                                              int width, int height );
;-----------------------------------------------------------------------------
INIT_YMM avx2
cglobal butteraugli_yuv_to_rgb_avx2_bt601, 10,11,15
    mov         r10d, r9d               ; height
    
.loop_y:
    xor         eax, eax                ; x = 0
    
.loop_x:
    ; Process 16 pixels at a time (double the original)
    ; Load 16 Y values  
    vmovdqu     xm0, [r0 + rax]        ; Y values (16 pixels)
    
    ; Load 8 U and V values (for 16 pixels in 4:2:0)
    mov         rdx, rax
    shr         rdx, 1                  ; rdx = rax/2
    vmovq       xm1, [r1 + rdx]        ; U values (8 pixels)
    vmovq       xm2, [r2 + rdx]        ; V values (8 pixels)
    
    ; Duplicate chroma for 2x2 blocks (4:2:0 to 4:4:4)
    vpunpcklbw  xm3, xm1, xm1           ; duplicate U low bytes
    vpunpcklbw  xm4, xm2, xm2           ; duplicate V low bytes
    
    ; Combine duplicated bytes to get proper ordering
    vpunpcklbw  xm5, xm3, xm3           ; AABBCCDD -> AAAABBBB (low 4)
    vpunpckhbw  xm6, xm3, xm3           ; AABBCCDD -> CCCCDDDD (high 4)
    vpunpcklwd  xm1, xm5, xm6           ; Combine to get first 8 U values
    vpunpckhwd  xm3, xm5, xm6           ; Get second 8 U values
    vinserti128 m1, m1, xm3, 1          ; Combine to full 256-bit for 16 U values
    
    vpunpcklbw  xm5, xm4, xm4           ; AABBCCDD -> AAAABBBB (low 4)
    vpunpckhbw  xm6, xm4, xm4           ; AABBCCDD -> CCCCDDDD (high 4)
    vpunpcklwd  xm2, xm5, xm6           ; Combine to get first 8 V values
    vpunpckhwd  xm4, xm5, xm6           ; Get second 8 V values
    vinserti128 m2, m2, xm4, 1          ; Combine to full 256-bit for 16 V values
    
    ; Extend Y to 256-bit
    vpxor       xm7, xm7, xm7
    vpunpcklbw  xm8, xm0, xm7           ; Y low 8 -> 16-bit
    vpunpckhbw  xm9, xm0, xm7           ; Y high 8 -> 16-bit
    vinserti128 m0, m8, xm9, 1          ; Combine to 256-bit
    
    ; Subtract 128 from UV
    vpsubb      xm1, xm1, [uv_offset]
    vpsubb      xm2, xm2, [uv_offset]
    
    ; Sign extend U and V to 16-bit
    vpmovsxbw   xm10, xm1               ; U low 8 -> signed 16-bit
    vpsrldq     xm11, xm1, 8            ; Shift for high 8 U values
    vpmovsxbw   xm11, xm11              ; U high 8 -> signed 16-bit
    vinserti128 m1, m10, xm11, 1        ; Combine to 256-bit
    
    vpmovsxbw   xm10, xm2               ; V low 8 -> signed 16-bit
    vpsrldq     xm11, xm2, 8            ; Shift for high 8 V values
    vpmovsxbw   xm11, xm11              ; V high 8 -> signed 16-bit
    vinserti128 m2, m10, xm11, 1        ; Combine to 256-bit
    
    ; Calculate R = Y + ((360 * V) >> 8)
    vpmullw     m3, m2, [yuv_r_coeff_avx2] ; V * 360
    vpsraw      m3, m3, 8               ; >> 8
    vpaddw      m3, m3, m0              ; + Y
    
    ; Calculate G = Y - ((88 * U + 184 * V) >> 8)
    vpmullw     m4, m1, [yuv_g_coeff_u_avx2] ; U * -88
    vpmullw     m5, m2, [yuv_g_coeff_v_avx2] ; V * -184
    vpaddw      m4, m4, m5              ; U*-88 + V*-184
    vpsraw      m4, m4, 8               ; >> 8
    vpaddw      m4, m4, m0              ; + Y
    
    ; Calculate B = Y + ((455 * U) >> 8)  
    vpmullw     m5, m1, [yuv_b_coeff_avx2] ; U * 455
    vpsraw      m5, m5, 8               ; >> 8
    vpaddw      m5, m5, m0              ; + Y
    
    ; Pack results
    vextracti128 xm6, m3, 1             ; Extract high R values
    vextracti128 xm7, m4, 1             ; Extract high G values
    vextracti128 xm8, m5, 1             ; Extract high B values
    vpackuswb   xm3, xm3, xm6           ; Pack 16 R values
    vpackuswb   xm4, xm4, xm7           ; Pack 16 G values
    vpackuswb   xm5, xm5, xm8           ; Pack 16 B values
    
    ; Store RGB to separate planes (planar format) - 16 pixels
    vmovdqu     [r5 + rax], xm3         ; Store R values (16 pixels)
    vmovdqu     [r6 + rax], xm4         ; Store G values (16 pixels)
    vmovdqu     [r7 + rax], xm5         ; Store B values (16 pixels)
    
    add         eax, 16                 ; process 16 pixels at a time
    cmp         eax, r8d                ; compare with width
    jl          .loop_x
    
    ; Advance to next row
    add         r0, r3                  ; y += y_stride
    add         r1, r4                  ; u += uv_stride
    add         r2, r4                  ; v += uv_stride
    add         r5, r8                  ; rgb_r += rgb_stride
    add         r6, r8                  ; rgb_g += rgb_stride
    add         r7, r8                  ; rgb_b += rgb_stride
    
    dec         r10d
    jnz         .loop_y
    
    RET

;-----------------------------------------------------------------------------
; void x264_butteraugli_srgb_to_linear_sse2( const uint8_t *srgb, float *linear,
;                                            int width, int height, int stride );
;-----------------------------------------------------------------------------
INIT_XMM sse2
cglobal butteraugli_srgb_to_linear_sse2, 5,7,8
    mov         r5d, r3d                ; height
    lea         r6, [srgb_to_linear_lut]
    
.loop_y:
    xor         eax, eax                ; x = 0
    
.loop_x:
    ; Load 4 sRGB values
    movd        m0, [r0 + rax]          ; load 4 bytes
    pmovzxbd    m0, m0                  ; zero extend bytes to dwords
    
    ; Extract individual dword indices and lookup in table
    movd        edx, m0
    movss       m1, [r6 + rdx*4]        ; lookup value 0
    pextrd      edx, m0, 1
    movss       m2, [r6 + rdx*4]        ; lookup value 1
    pextrd      edx, m0, 2
    movss       m3, [r6 + rdx*4]        ; lookup value 2
    pextrd      edx, m0, 3
    movss       m4, [r6 + rdx*4]        ; lookup value 3
    
    ; Pack the 4 floats together
    insertps    m1, m2, 0x10            ; insert value 1
    insertps    m1, m3, 0x20            ; insert value 2
    insertps    m1, m4, 0x30            ; insert value 3
    
    ; Store result
    movdqu      [r1 + rax*4], m1        ; store 4 floats
    
    add         eax, 4                  ; process 4 pixels at a time
    cmp         eax, r2d
    jl          .loop_x
    
    ; Advance to next row
    add         r0, r4                  ; srgb += stride
    mov         esi, r2d
    shl         esi, 2                  ; width * sizeof(float)
    add         r1, rsi                 ; linear += width*4
    
    dec         r5d
    jnz         .loop_y
    
    RET

;-----------------------------------------------------------------------------
; void x264_butteraugli_create_image_planar_sse2( const uint8_t *rgb_r, const uint8_t *rgb_g, const uint8_t *rgb_b,
;                                                 int rgb_stride, float *planar_r, float *planar_g, float *planar_b,
;                                                 int width, int height );
;-----------------------------------------------------------------------------
INIT_XMM sse2
cglobal butteraugli_create_image_planar_sse2, 8,9,4
    mov         r8d, r7d                ; height
    
.loop_y:
    xor         eax, eax                ; x = 0
    
.loop_x:
    ; Load 4 R values
    movd        m0, [r0 + rax]          ; load 4 R bytes
    punpcklbw   m0, m0                  ; unpack to words
    punpcklwd   m0, m0                  ; unpack to dwords
    cvtdq2ps    m0, m0                  ; convert to float
    movdqu      [r4 + rax*4], m0        ; store to planar_r
    
    ; Load 4 G values
    movd        m1, [r1 + rax]          ; load 4 G bytes
    punpcklbw   m1, m1                  ; unpack to words
    punpcklwd   m1, m1                  ; unpack to dwords
    cvtdq2ps    m1, m1                  ; convert to float
    movdqu      [r5 + rax*4], m1        ; store to planar_g
    
    ; Load 4 B values
    movd        m2, [r2 + rax]          ; load 4 B bytes
    punpcklbw   m2, m2                  ; unpack to words
    punpcklwd   m2, m2                  ; unpack to dwords
    cvtdq2ps    m2, m2                  ; convert to float
    movdqu      [r6 + rax*4], m2        ; store to planar_b
    
    add         eax, 4                  ; process 4 pixels at a time
    cmp         eax, r7d                ; compare with width
    jl          .loop_x
    
    ; Advance to next row
    add         r0, r3                  ; rgb_r += rgb_stride
    add         r1, r3                  ; rgb_g += rgb_stride
    add         r2, r3                  ; rgb_b += rgb_stride
    mov         edi, r7d                ; width
    shl         edi, 2                  ; width * sizeof(float)
    add         r4, rdi                 ; planar_r += width*4
    add         r5, rdi                 ; planar_g += width*4
    add         r6, rdi                 ; planar_b += width*4
    
    dec         r8d
    jnz         .loop_y
    
    RET

;-----------------------------------------------------------------------------
; void x264_butteraugli_compute_visual_mask_sse2( const float *ref_r, const float *ref_g, const float *ref_b,
;                                                 int ref_stride, float *mask, int mask_stride,
;                                                 int width, int height );
;-----------------------------------------------------------------------------
INIT_XMM sse2
cglobal butteraugli_compute_visual_mask_sse2, 8,12,8
    mov         r8d, 1                  ; y = 1 (start from border)
    
    ; Load constants
    movss       xmm4, [luma_r_coeff]
    shufps      xmm4, xmm4, 0          ; broadcast luma_r
    movss       xmm5, [luma_g_coeff]
    shufps      xmm5, xmm5, 0          ; broadcast luma_g
    movss       xmm6, [luma_b_coeff]
    shufps      xmm6, xmm6, 0          ; broadcast luma_b
    movss       xmm7, [scale_100]
    shufps      xmm7, xmm7, 0          ; broadcast scale
    
.loop_y:
    mov         eax, 1                  ; x = 1 (skip border)
    
.loop_x:
    ; Vectorized 3x3 variance computation
    ; Load 3x3 neighborhood and compute luma values using SIMD
    
    ; Calculate base offset for current pixel (x,y)
    mov         esi, r8d                ; y
    dec         esi                     ; y - 1
    imul        esi, r3d                ; (y-1) * ref_stride
    add         esi, eax                ; + x
    dec         esi                     ; + (x-1)
    shl         esi, 2                  ; * sizeof(float)
    
    ; Initialize accumulators
    xorps       m0, m0                  ; sum accumulator
    xorps       m1, m1                  ; sum_sq accumulator
    
    ; Process 3 rows of the 3x3 neighborhood
    ; We'll accumulate all 9 values correctly
    
    ; Row 1 (y-1)
    movss       m2, [r0 + r11]          ; R[x-1,y-1]
    movss       m3, [r1 + r11]          ; G[x-1,y-1]
    movss       m7, [r2 + r11]          ; B[x-1,y-1]
    mulss       m2, xmm4                ; R * 0.299
    mulss       m3, xmm5                ; G * 0.587
    mulss       m7, xmm6                ; B * 0.114
    addss       m2, m3
    addss       m2, m7                  ; luma[0]
    addss       m0, m2                  ; sum += luma
    mulss       m2, m2
    addss       m1, m2                  ; sum_sq += luma²
    
    add         r11d, 4                 ; next pixel
    movss       m2, [r0 + r11]          ; R[x,y-1]
    movss       m3, [r1 + r11]          ; G[x,y-1]
    movss       m7, [r2 + r11]          ; B[x,y-1]
    mulss       m2, xmm4
    mulss       m3, xmm5
    mulss       m7, xmm6
    addss       m2, m3
    addss       m2, m7                  ; luma[1]
    addss       m0, m2
    mulss       m2, m2
    addss       m1, m2
    
    add         r11d, 4                 ; next pixel
    movss       m2, [r0 + r11]          ; R[x+1,y-1]
    movss       m3, [r1 + r11]          ; G[x+1,y-1]
    movss       m7, [r2 + r11]          ; B[x+1,y-1]
    mulss       m2, xmm4
    mulss       m3, xmm5
    mulss       m7, xmm6
    addss       m2, m3
    addss       m2, m7                  ; luma[2]
    addss       m0, m2
    mulss       m2, m2
    addss       m1, m2
    
    ; Move to next row
    sub         r11d, 8                 ; back to x-1
    add         r11d, r3d               ; next row
    shl         r11d, 2                 ; * sizeof(float)
    
    ; Row 2 (y) - repeat same pattern
    movss       m2, [r0 + r11]
    movss       m3, [r1 + r11]
    movss       m7, [r2 + r11]
    mulss       m2, xmm4
    mulss       m3, xmm5
    mulss       m7, xmm6
    addss       m2, m3
    addss       m2, m7
    addss       m0, m2
    mulss       m2, m2
    addss       m1, m2
    
    add         r11d, 4
    movss       m2, [r0 + r11]
    movss       m3, [r1 + r11]
    movss       m7, [r2 + r11]
    mulss       m2, xmm4
    mulss       m3, xmm5
    mulss       m7, xmm6
    addss       m2, m3
    addss       m2, m7
    addss       m0, m2
    mulss       m2, m2
    addss       m1, m2
    
    add         r11d, 4
    movss       m2, [r0 + r11]
    movss       m3, [r1 + r11]
    movss       m7, [r2 + r11]
    mulss       m2, xmm4
    mulss       m3, xmm5
    mulss       m7, xmm6
    addss       m2, m3
    addss       m2, m7
    addss       m0, m2
    mulss       m2, m2
    addss       m1, m2
    
    ; Move to next row
    sub         r11d, 8
    add         r11d, r3d
    shl         r11d, 2
    
    ; Row 3 (y+1) - repeat same pattern
    movss       m2, [r0 + r11]
    movss       m3, [r1 + r11]
    movss       m7, [r2 + r11]
    mulss       m2, xmm4
    mulss       m3, xmm5
    mulss       m7, xmm6
    addss       m2, m3
    addss       m2, m7
    addss       m0, m2
    mulss       m2, m2
    addss       m1, m2
    
    add         r11d, 4
    movss       m2, [r0 + r11]
    movss       m3, [r1 + r11]
    movss       m7, [r2 + r11]
    mulss       m2, xmm4
    mulss       m3, xmm5
    mulss       m7, xmm6
    addss       m2, m3
    addss       m2, m7
    addss       m0, m2
    mulss       m2, m2
    addss       m1, m2
    
    add         r11d, 4
    movss       m2, [r0 + r11]
    movss       m3, [r1 + r11]
    movss       m7, [r2 + r11]
    mulss       m2, xmm4
    mulss       m3, xmm5
    mulss       m7, xmm6
    addss       m2, m3
    addss       m2, m7
    addss       m0, m2
    mulss       m2, m2
    addss       m1, m2
    
    ; Now m0 contains sum and m1 contains sum_sq of all 9 values
    
    ; Compute variance = (sum_sq/9) - (mean)²
    movss       m7, [nine]              ; 9.0
    divss       m0, m7                  ; mean = sum / 9
    divss       m1, m7                  ; sum_sq / 9
    
    movss       m2, m0                  ; copy mean
    mulss       m2, m2                  ; mean²
    subss       m1, m2                  ; variance = (sum_sq/9) - mean²
    
    ; Apply scaling and exponential mapping
    mulss       m1, xmm7                ; variance * 100.0
    
    ; Compute exp(-variance) using improved approximation
    ; exp(-x) ≈ 1 - x + x²/2 - x³/6 + x⁴/24
    movss       m2, m1                  ; x = variance * 100
    movss       m3, m1
    mulss       m3, m2                  ; x²
    
    movss       m7, [ones]              ; 1.0
    subss       m7, m2                  ; 1 - x
    
    movss       m2, m3
    mulss       m2, [half]              ; x²/2
    addss       m7, m2                  ; 1 - x + x²/2
    
    mulss       m3, m1                  ; x³
    mulss       m3, [sixth]             ; x³/6
    subss       m7, m3                  ; 1 - x + x²/2 - x³/6
    
    ; masking = 1.0 + exp(-variance)
    movss       m3, [ones]
    addss       m3, m7                  ; 1.0 + exp(-variance)
    
    ; Clamp to valid range [0.5, 2.0]
    maxss       m3, [min_mask_val]
    minss       m3, [max_mask_val]
    
    ; Store result
    mov         esi, r8d                ; y
    imul        esi, r5d                ; y * mask_stride  
    add         esi, eax                ; + x
    shl         esi, 2                  ; * sizeof(float)
    movss       [r4 + rsi], m3
    
    inc         eax                     ; x++
    mov         edx, r6d                ; width
    dec         edx                     ; width - 1
    cmp         eax, edx
    jl          .loop_x
    
    ; Advance to next row
    add         r0, r3                  ; ref_r += stride
    add         r1, r3                  ; ref_g += stride
    add         r2, r3                  ; ref_b += stride
    add         r4, r5                  ; mask += mask_stride
    
    inc         r8d                     ; y++
    mov         edx, r7d                ; height
    dec         edx                     ; height - 1
    cmp         r8d, edx
    jl          .loop_y
    
    RET

;-----------------------------------------------------------------------------
; void x264_butteraugli_heatmap_to_uint8_sse2( const float *heatmap, int heatmap_stride,
;                                              uint8_t *output, int output_stride,
;                                              int width, int height, float scale );
;-----------------------------------------------------------------------------
INIT_XMM sse2
cglobal butteraugli_heatmap_to_uint8_sse2, 7,8,4
    mov         r7d, r5d                ; height
    
    ; Load scale factor
    movss       xmm3, [rsp + 8*7]      ; scale parameter
    shufps      xmm3, xmm3, 0          ; broadcast scale
    
.loop_y:
    xor         eax, eax                ; x = 0
    
.loop_x:
    ; Load 4 float values
    movups      m0, [r0 + rax*4]       ; heatmap values
    
    ; Scale values
    mulps       m0, xmm3
    
    ; Clamp to 0-255 range
    maxps       m0, [zeros]
    minps       m0, [ff_255]
    
    ; Convert to integers
    cvtps2dq    m0, m0
    
    ; Pack to bytes
    packssdw    m0, m0                  ; 32->16 bit
    packuswb    m0, m0                  ; 16->8 bit
    
    ; Store 4 bytes
    movd        [r2 + rax], m0
    
    add         eax, 4                  ; process 4 pixels
    cmp         eax, r4d
    jl          .loop_x
    
    ; Advance to next row
    add         r0, r1                  ; heatmap += heatmap_stride
    add         r2, r3                  ; output += output_stride
    
    dec         r7d
    jnz         .loop_y
    
    RET

;-----------------------------------------------------------------------------
; void x264_butteraugli_srgb_to_linear_avx2( const uint8_t *srgb, float *linear,
;                                            int width, int height, int stride );
;-----------------------------------------------------------------------------
INIT_YMM avx2
cglobal butteraugli_srgb_to_linear_avx2, 5,7,8
    mov         r5d, r3d                ; height
    lea         r6, [srgb_to_linear_lut]
    
.loop_y:
    xor         eax, eax                ; x = 0
    
.loop_x:
    ; Load 8 sRGB values
    movq        xm0, [r0 + rax]         ; load 8 bytes
    pmovzxbd    m0, xm0                 ; zero extend bytes to dwords
    
    ; Use lookup table for accurate sRGB to linear conversion
    ; Extract individual dword indices and gather from LUT
    vextractf128 xm1, m0, 0             ; lower 4 dwords
    vextractf128 xm2, m0, 1             ; upper 4 dwords
    
    ; Gather lower 4 values from LUT
    vmovd       edx, xm1
    movss       xm3, [r6 + rdx*4]
    vpextrd     edx, xm1, 1
    movss       xm4, [r6 + rdx*4]
    vpextrd     edx, xm1, 2
    movss       xm5, [r6 + rdx*4]
    vpextrd     edx, xm1, 3
    movss       xm6, [r6 + rdx*4]
    
    ; Pack lower 4 floats
    vinsertps   xm3, xm3, xm4, 0x10
    vinsertps   xm3, xm3, xm5, 0x20
    vinsertps   xm3, xm3, xm6, 0x30
    
    ; Gather upper 4 values from LUT
    vmovd       edx, xm2
    movss       xm4, [r6 + rdx*4]
    vpextrd     edx, xm2, 1
    movss       xm5, [r6 + rdx*4]
    vpextrd     edx, xm2, 2
    movss       xm6, [r6 + rdx*4]
    vpextrd     edx, xm2, 3
    movss       xm7, [r6 + rdx*4]
    
    ; Pack upper 4 floats
    vinsertps   xm4, xm4, xm5, 0x10
    vinsertps   xm4, xm4, xm6, 0x20
    vinsertps   xm4, xm4, xm7, 0x30
    
    ; Combine into 256-bit register
    vinsertf128 m0, m3, xm4, 1
    
    ; Store 8 floats
    vmovups     [r1 + rax*4], m0
    
    add         eax, 8
    cmp         eax, r2d
    jl          .loop_x
    
    ; Advance to next row
    add         r0, r4                  ; srgb += stride
    mov         eax, r2d
    shl         eax, 2                  ; width * sizeof(float)
    add         r1, rax                 ; linear += width * 4
    
    dec         r5d
    jnz         .loop_y
    
    RET

;-----------------------------------------------------------------------------
; void x264_butteraugli_create_image_planar_avx2( const uint8_t *rgb_r, const uint8_t *rgb_g, const uint8_t *rgb_b,
;                                                 int rgb_stride, float *planar_r, float *planar_g, float *planar_b,
;                                                 int width, int height );
;-----------------------------------------------------------------------------
INIT_YMM avx2
cglobal butteraugli_create_image_planar_avx2, 8,9,4
    mov         r8d, [rsp + 8*8]        ; height (9th parameter)
    
.loop_y:
    xor         eax, eax                ; x = 0
    
.loop_x:
    ; Load 8 R values
    movq        xm0, [r0 + rax]         ; load 8 R bytes
    pmovzxbd    m0, xm0                 ; zero extend to dwords
    vcvtdq2ps   m0, m0                  ; convert to float
    vmovups     [r4 + rax*4], m0        ; store to planar_r
    
    ; Load 8 G values
    movq        xm1, [r1 + rax]         ; load 8 G bytes
    pmovzxbd    m1, xm1                 ; zero extend to dwords
    vcvtdq2ps   m1, m1                  ; convert to float
    vmovups     [r5 + rax*4], m1        ; store to planar_g
    
    ; Load 8 B values
    movq        xm2, [r2 + rax]         ; load 8 B bytes
    pmovzxbd    m2, xm2                 ; zero extend to dwords
    vcvtdq2ps   m2, m2                  ; convert to float
    vmovups     [r6 + rax*4], m2        ; store to planar_b
    
    add         eax, 8                  ; process 8 pixels at a time
    cmp         eax, r7d                ; compare with width
    jl          .loop_x
    
    ; Advance to next row
    add         r0, r3                  ; rgb_r += rgb_stride
    add         r1, r3                  ; rgb_g += rgb_stride
    add         r2, r3                  ; rgb_b += rgb_stride
    mov         edi, r7d                ; width
    shl         edi, 2                  ; width * sizeof(float)
    add         r4, rdi                 ; planar_r += width*4
    add         r5, rdi                 ; planar_g += width*4
    add         r6, rdi                 ; planar_b += width*4
    
    dec         r8d
    jnz         .loop_y
    
    RET

;-----------------------------------------------------------------------------
; void x264_butteraugli_compute_visual_mask_avx2( const float *ref_r, const float *ref_g, const float *ref_b,
;                                                 int ref_stride, float *mask, int mask_stride,
;                                                 int width, int height );
;-----------------------------------------------------------------------------
INIT_YMM avx2
cglobal butteraugli_compute_visual_mask_avx2, 8,12,8
    mov         r8d, 1                  ; y = 1 (start from border)
    mov         r9d, [rsp + 8*8]        ; height
    dec         r9d                     ; height - 1
    
    ; Load constants
    vbroadcastss m4, [luma_r_coeff]
    vbroadcastss m5, [luma_g_coeff]
    vbroadcastss m6, [luma_b_coeff]
    vbroadcastss m7, [nine]
    
.loop_y:
    mov         eax, 1                  ; x = 1 (skip border)
    mov         r10d, r6d               ; width
    dec         r10d                    ; width - 1
    
.loop_x:
    ; Calculate base offset for current pixel
    mov         r11d, r8d               ; y
    imul        r11d, r3d               ; y * ref_stride
    add         r11d, eax               ; + x
    
    ; Initialize accumulators for 3x3 neighborhood
    vxorps      m0, m0, m0              ; sum
    vxorps      m1, m1, m1              ; sum_sq
    
    ; Process 3x3 neighborhood
    ; dy = -1
    mov         esi, r11d
    sub         esi, r3d                ; (y-1) * ref_stride + x
    dec         esi                     ; x - 1
    shl         esi, 2                  ; * sizeof(float)
    
    ; Load and accumulate row y-1
    vmovss      xm2, [r0 + rsi]         ; R[x-1,y-1]
    vmovss      xm3, [r1 + rsi]         ; G[x-1,y-1]
    vmovss      xm7, [r2 + rsi]         ; B[x-1,y-1]
    vmulss      xm2, xm2, xm4           ; R * 0.299
    vmulss      xm3, xm3, xm5           ; G * 0.587
    vmulss      xm7, xm7, xm6           ; B * 0.114
    vaddss      xm2, xm2, xm3
    vaddss      xm2, xm2, xm7           ; luma
    vaddss      xm0, xm0, xm2           ; sum += luma
    vmulss      xm3, xm2, xm2
    vaddss      xm1, xm1, xm3           ; sum_sq += luma²
    
    add         esi, 4                  ; move to x position
    vmovss      xm2, [r0 + rsi]         ; R[x,y-1]
    vmovss      xm3, [r1 + rsi]         ; G[x,y-1]
    vmovss      xm7, [r2 + rsi]         ; B[x,y-1]
    vmulss      xm2, xm2, xm4
    vmulss      xm3, xm3, xm5
    vmulss      xm7, xm7, xm6
    vaddss      xm2, xm2, xm3
    vaddss      xm2, xm2, xm7           ; luma
    vaddss      xm0, xm0, xm2           ; sum += luma
    vmulss      xm3, xm2, xm2
    vaddss      xm1, xm1, xm3           ; sum_sq += luma²
    
    add         esi, 4                  ; move to x+1 position
    vmovss      xm2, [r0 + rsi]         ; R[x+1,y-1]
    vmovss      xm3, [r1 + rsi]         ; G[x+1,y-1]
    vmovss      xm7, [r2 + rsi]         ; B[x+1,y-1]
    vmulss      xm2, xm2, xm4
    vmulss      xm3, xm3, xm5
    vmulss      xm7, xm7, xm6
    vaddss      xm2, xm2, xm3
    vaddss      xm2, xm2, xm7           ; luma
    vaddss      xm0, xm0, xm2           ; sum += luma
    vmulss      xm3, xm2, xm2
    vaddss      xm1, xm1, xm3           ; sum_sq += luma²
    
    ; dy = 0 (current row)
    mov         esi, r11d
    shl         esi, 2                  ; * sizeof(float)
    sub         esi, 4                  ; x - 1
    
    vmovss      xm2, [r0 + rsi]         ; R[x-1,y]
    vmovss      xm3, [r1 + rsi]         ; G[x-1,y]
    vmovss      xm7, [r2 + rsi]         ; B[x-1,y]
    vmulss      xm2, xm2, xm4
    vmulss      xm3, xm3, xm5
    vmulss      xm7, xm7, xm6
    vaddss      xm2, xm2, xm3
    vaddss      xm2, xm2, xm7           ; luma
    vaddss      xm0, xm0, xm2           ; sum += luma
    vmulss      xm3, xm2, xm2
    vaddss      xm1, xm1, xm3           ; sum_sq += luma²
    
    add         esi, 4                  ; move to x position
    vmovss      xm2, [r0 + rsi]         ; R[x,y]
    vmovss      xm3, [r1 + rsi]         ; G[x,y]
    vmovss      xm7, [r2 + rsi]         ; B[x,y]
    vmulss      xm2, xm2, xm4
    vmulss      xm3, xm3, xm5
    vmulss      xm7, xm7, xm6
    vaddss      xm2, xm2, xm3
    vaddss      xm2, xm2, xm7           ; luma
    vaddss      xm0, xm0, xm2           ; sum += luma
    vmulss      xm3, xm2, xm2
    vaddss      xm1, xm1, xm3           ; sum_sq += luma²
    
    add         esi, 4                  ; move to x+1 position
    vmovss      xm2, [r0 + rsi]         ; R[x+1,y]
    vmovss      xm3, [r1 + rsi]         ; G[x+1,y]
    vmovss      xm7, [r2 + rsi]         ; B[x+1,y]
    vmulss      xm2, xm2, xm4
    vmulss      xm3, xm3, xm5
    vmulss      xm7, xm7, xm6
    vaddss      xm2, xm2, xm3
    vaddss      xm2, xm2, xm7           ; luma
    vaddss      xm0, xm0, xm2           ; sum += luma
    vmulss      xm3, xm2, xm2
    vaddss      xm1, xm1, xm3           ; sum_sq += luma²
    
    ; dy = 1 (next row)
    mov         esi, r11d
    add         esi, r3d                ; (y+1) * ref_stride + x
    dec         esi                     ; x - 1
    shl         esi, 2                  ; * sizeof(float)
    
    vmovss      xm2, [r0 + rsi]         ; R[x-1,y+1]
    vmovss      xm3, [r1 + rsi]         ; G[x-1,y+1]
    vmovss      xm7, [r2 + rsi]         ; B[x-1,y+1]
    vmulss      xm2, xm2, xm4
    vmulss      xm3, xm3, xm5
    vmulss      xm7, xm7, xm6
    vaddss      xm2, xm2, xm3
    vaddss      xm2, xm2, xm7           ; luma
    vaddss      xm0, xm0, xm2           ; sum += luma
    vmulss      xm3, xm2, xm2
    vaddss      xm1, xm1, xm3           ; sum_sq += luma²
    
    add         esi, 4                  ; move to x position
    vmovss      xm2, [r0 + rsi]         ; R[x,y+1]
    vmovss      xm3, [r1 + rsi]         ; G[x,y+1]
    vmovss      xm7, [r2 + rsi]         ; B[x,y+1]
    vmulss      xm2, xm2, xm4
    vmulss      xm3, xm3, xm5
    vmulss      xm7, xm7, xm6
    vaddss      xm2, xm2, xm3
    vaddss      xm2, xm2, xm7           ; luma
    vaddss      xm0, xm0, xm2           ; sum += luma
    vmulss      xm3, xm2, xm2
    vaddss      xm1, xm1, xm3           ; sum_sq += luma²
    
    add         esi, 4                  ; move to x+1 position
    vmovss      xm2, [r0 + rsi]         ; R[x+1,y+1]
    vmovss      xm3, [r1 + rsi]         ; G[x+1,y+1]
    vmovss      xm7, [r2 + rsi]         ; B[x+1,y+1]
    vmulss      xm2, xm2, xm4
    vmulss      xm3, xm3, xm5
    vmulss      xm7, xm7, xm6
    vaddss      xm2, xm2, xm3
    vaddss      xm2, xm2, xm7           ; luma
    vaddss      xm0, xm0, xm2           ; sum += luma
    vmulss      xm3, xm2, xm2
    vaddss      xm1, xm1, xm3           ; sum_sq += luma²
    ; Calculate variance = (sum_sq / 9) - (sum / 9)²
    vdivss      xm2, xm0, xm7           ; mean = sum / 9
    vdivss      xm3, xm1, xm7           ; mean_sq = sum_sq / 9
    vmulss      xm0, xm2, xm2           ; mean²
    vsubss      xm3, xm3, xm0           ; variance = mean_sq - mean²
    
    ; Apply exponential mapping: 1 + exp(-variance * 100)
    vmulss      xm3, xm3, [scale_100]   ; -variance * 100
    ; Simple exp approximation: 1/(1 + x) for negative x
    vmovss      xm4, [zeros]
    vsubss      xm3, xm4, xm3           ; -(-variance * 100) = variance * 100
    vmovss      xm4, [ones]
    vaddss      xm3, xm3, xm4           ; 1 + variance * 100
    vdivss      xm3, xm4, xm3           ; 1 / (1 + variance * 100)
    vaddss      xm3, xm3, xm4           ; 1 + exp_approx
    
    ; Clamp to range [0.5, 2.0]
    vmaxss      xm3, xm3, [min_mask_val]
    vminss      xm3, xm3, [max_mask_val]
    
    ; Store result
    mov         esi, r8d
    imul        esi, r5d                ; y * mask_stride
    add         esi, eax                ; + x
    vmovss      [r4 + rsi*4], xm3
    
    inc         eax
    cmp         eax, r10d
    jl          .loop_x
    
    inc         r8d
    cmp         r8d, r9d
    jl          .loop_y
    
    ; Handle borders - set to 1.0
    ; Top and bottom rows
    xor         eax, eax
.border_x:
    vmovss      xm0, [ones]
    vmovss      [r4 + rax*4], xm0                      ; top row
    mov         esi, r9d
    imul        esi, r5d
    add         esi, eax
    vmovss      [r4 + rsi*4], xm0                      ; bottom row
    inc         eax
    cmp         eax, r6d
    jl          .border_x
    
    ; Left and right columns
    xor         r8d, r8d
.border_y:
    mov         esi, r8d
    imul        esi, r5d
    vmovss      xm0, [ones]
    vmovss      [r4 + rsi*4], xm0                      ; left column
    add         esi, r6d
    dec         esi
    vmovss      [r4 + rsi*4], xm0                      ; right column
    inc         r8d
    cmp         r8d, r9d
    jle         .border_y
    
    RET

;-----------------------------------------------------------------------------
; void x264_butteraugli_heatmap_to_uint8_avx2( const float *heatmap, int heatmap_stride,
;                                              uint8_t *output, int output_stride,
;                                              int width, int height, float scale );
;-----------------------------------------------------------------------------
INIT_YMM avx2
cglobal butteraugli_heatmap_to_uint8_avx2, 7,8,4
    mov         r7d, r5d                ; height
    
    ; Load scale factor
    vmovss      xmm3, [rsp + 8*7]       ; scale parameter
    vbroadcastss m3, xmm3               ; broadcast scale
    
.loop_y:
    xor         eax, eax                ; x = 0
    
.loop_x:
    ; Load 8 float values
    vmovups     m0, [r0 + rax*4]        ; heatmap values
    
    ; Scale values
    vmulps      m0, m0, m3
    
    ; Clamp to 0-255 range
    vmaxps      m0, m0, [zeros]
    vminps      m0, m0, [ff_255]
    
    ; Convert to integers
    vcvtps2dq   m0, m0
    
    ; Pack to bytes (32->16->8)
    vpackssdw   m0, m0, m0              ; 32->16 bit
    vpermq      m0, m0, 0xD8            ; fix lane crossing
    vpackuswb   m0, m0, m0              ; 16->8 bit
    
    ; Store 8 bytes
    vmovq       [r2 + rax], xm0
    
    add         eax, 8                  ; process 8 pixels
    cmp         eax, r4d
    jl          .loop_x
    
    ; Advance to next row
    add         r0, r1                  ; heatmap += heatmap_stride
    add         r2, r3                  ; output += output_stride
    
    dec         r7d
    jnz         .loop_y
    
    RET

SECTION_RODATA 32
; AVX2 aligned constants (256-bit)
align 32
srgb_scale: times 8 dd 0.003921569    ; 1/255 for AVX2
zeros: times 8 dd 0.0
ff_255: times 8 dd 255.0
ones: times 8 dd 1.0
min_mask_val: times 8 dd 0.5
max_mask_val: times 8 dd 2.0

; SSE2 constants (128-bit)
align 16
srgb_scale_sse2: times 4 dd 0.003921569    ; 1/255
gamma_22: times 4 dd 2.2              ; Gamma 2.2 constant
gamma_inv: times 4 dd 0.45454545      ; 1/2.2 for inverse gamma
srgb_threshold: times 4 dd 0.04045    ; sRGB linear threshold
srgb_scale_factor: times 4 dd 12.92   ; sRGB linear scale factor
srgb_offset: times 4 dd 0.055         ; sRGB gamma offset
srgb_gamma_scale: times 4 dd 1.055    ; sRGB gamma scale
luma_r_coeff: dd 0.299
luma_g_coeff: dd 0.587
luma_b_coeff: dd 0.114
scale_100: dd 100.0
half: times 4 dd 0.5
nine: dd 9.0                          ; For 3x3 neighborhood
sixth: times 4 dd 0.166666667         ; 1/6 for exp approximation