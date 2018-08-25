/*
 * Copyright (c) 2018 Timo Sandmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    atomic.h
 * @brief   Helper functions for atomic execution
 * @author  Timo Sandmann
 * @date    25.08.2018
 */

#ifndef _ARDUINO_UTIL_ATOMIC_H_
#define _ARDUINO_UTIL_ATOMIC_H_

#define ATOMIC_BLOCK(type)

#define ATOMIC_RESTORESTATE 0

#define ATOMIC_FORCEON 1

#define NONATOMIC_BLOCK(type)

#define NONATOMIC_RESTORESTATE 2

#define NONATOMIC_FORCEOFF 3

#endif /* _ARDUINO_UTIL_ATOMIC_H_ */