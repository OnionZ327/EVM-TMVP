/* ====================================================================================================================

The copyright in this software is being made available under the License included below.
This software may be subject to other third party and contributor rights, including patent rights, and no such
rights are granted under this license.

Copyright (c) 2018, HUAWEI TECHNOLOGIES CO., LTD. All rights reserved.
Copyright (c) 2018, SAMSUNG ELECTRONICS CO., LTD. All rights reserved.
Copyright (c) 2018, PEKING UNIVERSITY SHENZHEN GRADUATE SCHOOL. All rights reserved.
Copyright (c) 2018, PENGCHENG LABORATORY. All rights reserved.
Copyright (c) 2019, TENCENT CO., LTD. All rights reserved.
Copyright (c) 2019, WUHAN UNIVERSITY. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted only for
the purpose of developing standards within Audio and Video Coding Standard Workgroup of China (AVS) and for testing and
promoting such standards. The following conditions are required to be met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and
the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
the following disclaimer in the documentation and/or other materials provided with the distribution.
* The name of HUAWEI TECHNOLOGIES CO., LTD. or SAMSUNG ELECTRONICS CO., LTD. or TENCENT CO., LTD. may not be used to endorse or promote products derived from
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

/** \file     enc_ibc_hashmap.h
    \brief    IBC hash map encoder class (header)
*/

#ifndef __ENC_RSD_HASHMAP__
#define __ENC_RSD_HASHMAP__

#include "enc_def.h"
#undef max
#undef min
#include <unordered_map>
#include <vector>
#if RSD_OPT
// ====================================================================================================================
// Class definition
// ====================================================================================================================
typedef std::pair<int, int> PositionRsd;

class ENC_RSD_HashMap
{
private:
    int     m_picWidth;
    int     m_picHeight;

    unsigned int xxCalcBlockHashRsd(const pel* pel, const int stride, const int width, const int height, unsigned int crc);
    void         xxBuildPicHashMapRsd(const COM_PIC* pic);

    static  unsigned int xxComputeCrc32c16bitRsd(unsigned int crc, const pel pel);

public:
    unsigned int** m_pos2HashRsd;
    std::unordered_map<unsigned int, std::vector<PositionRsd>> m_hash2PosRsd;

public:
    unsigned int(*m_computeCrc32cRsd) (unsigned int crc, const pel pel);

    ENC_RSD_HashMap();
    virtual ~ENC_RSD_HashMap();

    void    init(const int picWidth, const int picHeight);
    void    destroy();
    void    rebuildPicHashMapRsd(const COM_PIC* pic);
    void    buildFlipHashTable(int cu_x, int cu_y, int log2_cuw, int log2_cuh,
        unsigned int pos2HashCurPU[MAX_CU_SIZE][MAX_CU_SIZE], const int sp_rsd_flag, const COM_PIC* pic);
    bool    rsdHashMatch(ENC_CTX* ctx, int cu_x, int cu_y, int log2_cuw, int log2_cuh,
        std::vector<PositionRsd>& cand, const int maxCand, const int searchRange4SmallBlk,int intra_row_rsd_flag, const COM_PIC* pic);
};
#endif
#endif // __ENC_IBC_HASHMAP__
