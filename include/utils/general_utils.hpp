/********************************************************************************************
 *  \file       utils.hpp
 *  \brief      An state estimation server for AeroStack2
 *  \authors    David Pérez Saura
 *              Miguel Fernández Cortizas
 *              Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef __AS2__SEMANTIC_SLAM_UTILS_HPP_
#define __AS2__SEMANTIC_SLAM_UTILS_HPP_

#define RESET_COLOR "\033[0m"
#define RED_COLOR "\033[0;31m"
#define GREEN_COLOR "\033[1;32m"
#define YELLOW_COLOR "\033[1;33m"
#define MAGENTA_COLOR "\033[0;35m"
#define CYAN_COLOR "\033[0;36m"
#define ERROR(x)                                                                        \
  std::cerr << RED_COLOR << "[ERROR] " << x << std::endl                                \
            << "\t  at line " << __LINE__ << " in function " << __func__ << RESET_COLOR \
            << std::endl

#define WARN(x) std::cout << YELLOW_COLOR << "[WARN] " << x << RESET_COLOR << std::endl

#define FLAG(x) std::cout << CYAN_COLOR << "[FLAG] " << x << RESET_COLOR << std::endl
#define DEBUG(x)                                                                        \
  std::cout << YELLOW_COLOR << "[DEBUG] " << x << std::endl                             \
            << "\t  at line " << __LINE__ << " in function " << __func__ << RESET_COLOR \
            << std::endl
#define INFO(x) std::cout << x << std::endl;

#endif  // __AS2__SEMANTIC_SLAM_UTILS_HPP_
