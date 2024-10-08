/* ====================================================================================================================

  The copyright in this software is being made available under the License included below.
  This software may be subject to other third party and contributor rights, including patent rights, and no such
  rights are granted under this license.

  Copyright (c) 2018, HUAWEI TECHNOLOGIES CO., LTD. All rights reserved.
  Copyright (c) 2018, SAMSUNG ELECTRONICS CO., LTD. All rights reserved.
  Copyright (c) 2018, PEKING UNIVERSITY SHENZHEN GRADUATE SCHOOL. All rights reserved.
  Copyright (c) 2018, PENGCHENG LABORATORY. All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, are permitted only for
  the purpose of developing standards within Audio and Video Coding Standard Workgroup of China (AVS) and for testing and
  promoting such standards. The following conditions are required to be met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
      the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
      the following disclaimer in the documentation and/or other materials provided with the distribution.
    * The name of HUAWEI TECHNOLOGIES CO., LTD. or SAMSUNG ELECTRONICS CO., LTD. may not be used to endorse or promote products derived from
      this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

* ====================================================================================================================
*/

#ifndef _COM_ITDQ_H_
#define _COM_ITDQ_H_

void com_itdq(COM_MODE *mode, int plane, int blk_idx, s16 * coef, s16 *resi, u8* wq[2], int log2_w, int log2_h, int qp, int bit_depth, int secT_Ver_Hor, int use_alt4x4Trans
#if IST_CHROMA
    , CHANNEL_TYPE channel
#endif
);

#if ST_CHROMA
void com_itdq_yuv(COM_MODE* mode, s16 coef[N_C][MAX_CU_DIM], s16 resi[N_C][MAX_CU_DIM], u8* wq[2], int cu_width_log2, int cu_height_log2, int qp_y, int qp_u, int qp_v, int bit_depth, int secT_Ver_Hor[MAX_NUM_CHANNEL][MAX_NUM_TB], int use_alt4x4Trans[MAX_NUM_CHANNEL]
#if IST_CHROMA
    , CHANNEL_TYPE channel
#endif
);
#else
void com_itdq_yuv(COM_MODE* mode, s16 coef[N_C][MAX_CU_DIM], s16 resi[N_C][MAX_CU_DIM], u8* wq[2], int cu_width_log2, int cu_height_log2, int qp_y, int qp_u, int qp_v, int bit_depth, int secT_Ver_Hor[MAX_NUM_TB], int use_alt4x4Trans);
#endif

void com_itdq_plane(COM_MODE *mode, int plane, s16 coef[MAX_CU_DIM], s16 resi[MAX_CU_DIM], u8* wq[2], int cu_width_log2, int cu_height_log2, int qp, int bit_depth, int secT_Ver_Hor[MAX_NUM_TB], int use_alt4x4Trans
#if IST_CHROMA
    , CHANNEL_TYPE channel
#endif
);

#endif /* _COM_ITDQ_H_ */
