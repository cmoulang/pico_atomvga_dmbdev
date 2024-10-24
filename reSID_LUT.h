/*
       ______/  _____/  _____/     /   _/    /             /
     _/           /     /     /   /  _/     /   ______/   /  _/             ____/     /   ______/   ____/
      ___/       /     /     /   ___/      /   /         __/                    _/   /   /         /     /
         _/    _/    _/    _/   /  _/     /  _/         /  _/             _____/    /  _/        _/    _/
  ______/   _____/  ______/   _/    _/  _/    _____/  _/    _/          _/        _/    _____/    ____/
																  
  reSID_LUT.h

  SIDKick pico - SID-replacement with dual-SID/SID+fm emulation using a RPi pico, reSID 0.16 and fmopl 
  Copyright (c) 2023/2024 Carsten Dachsbacher <frenetic@dachsbacher.de>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Generated by bin2c, do not edit manually */

/* Contents of file reSID_LUTs.exo */
unsigned char reSID_LUTs[32768];

const long int reSID_LUTs_exo_size = 1875;
const unsigned char reSID_LUTs_exo[1875] = {
    0x01, 0x00, 0x7E, 0x98, 0x01, 0x00, 0xFC, 0x03, 0x88, 0xD6, 0xAB, 0xEF, 0xEE, 0x03, 0xF0, 0x30,
    0x30, 0x15, 0x68, 0x33, 0x7E, 0xCD, 0xDE, 0x34, 0x00, 0x7E, 0x90, 0x15, 0x19, 0x60, 0x7E, 0x32,
    0x09, 0x05, 0x07, 0xFE, 0x08, 0xDE, 0x1B, 0xF0, 0xE4, 0x02, 0x1F, 0x7E, 0x61, 0xCD, 0x1F, 0x2F,
    0x10, 0xFA, 0x40, 0x37, 0xA8, 0xDF, 0xEB, 0x80, 0x64, 0x30, 0x38, 0x75, 0x3E, 0xA8, 0xDF, 0x83,
    0x01, 0xE0, 0x83, 0x4F, 0x50, 0xE8, 0x03, 0x57, 0xA1, 0x7D, 0x5B, 0xB0, 0x5D, 0x87, 0x7E, 0xAF,
    0x30, 0x82, 0x35, 0x5C, 0xBA, 0x5F, 0x2B, 0xF4, 0x81, 0x67, 0xBB, 0x50, 0x91, 0x0F, 0x6B, 0xF6,
    0x6D, 0x7E, 0x10, 0x3C, 0x6E, 0x47, 0xD3, 0x8C, 0x15, 0x14, 0x60, 0x2A, 0xFE, 0x94, 0xF8, 0x02,
    0xA7, 0xC6, 0x52, 0x73, 0x9E, 0x55, 0x91, 0x40, 0x86, 0x1E, 0x75, 0xD6, 0x15, 0x76, 0x47, 0x53,
    0xC6, 0x0A, 0x0A, 0x70, 0xD5, 0xEF, 0x9E, 0x27, 0x72, 0xF0, 0xBB, 0x8E, 0x89, 0x0B, 0x53, 0x7A,
    0x8D, 0xCC, 0xA4, 0x64, 0x78, 0xA1, 0x7B, 0x2B, 0x94, 0x29, 0xE5, 0x26, 0xBF, 0x86, 0x9B, 0x68,
    0x40, 0xA6, 0x70, 0x43, 0x78, 0x05, 0x7D, 0x55, 0xE1, 0x16, 0x19, 0x8A, 0x27, 0x9E, 0xDF, 0x86,
    0x4F, 0x74, 0x7C, 0xFD, 0x7E, 0x7E, 0xE4, 0xFF, 0x06, 0x20, 0x38, 0x3F, 0x38, 0xE4, 0x03, 0xEE,
    0xCC, 0x5F, 0x48, 0x68, 0x03, 0x97, 0x27, 0x23, 0x9A, 0xFE, 0xCE, 0xC3, 0xC0, 0x89, 0x11, 0x4D,
    0xFE, 0xE7, 0x0F, 0xD7, 0x1C, 0xB3, 0xC5, 0x61, 0x70, 0x7B, 0xDA, 0x4E, 0x65, 0x02, 0xCA, 0x7C,
    0x60, 0x70, 0x78, 0x7D, 0x23, 0x6F, 0x95, 0x36, 0x93, 0x78, 0x4D, 0x70, 0xFF, 0x4B, 0xBF, 0xFE,
    0xCE, 0xFF, 0x02, 0x9C, 0x3E, 0x4E, 0xEB, 0x9F, 0xBF, 0x41, 0xD2, 0x0B, 0xBE, 0x9C, 0x33, 0x3C,
    0xA0, 0x57, 0xAF, 0x9F, 0x8E, 0x2C, 0xA0, 0x65, 0x4E, 0xA0, 0x8C, 0xB0, 0xB7, 0x4E, 0x9F, 0xC3,
    0xCA, 0x63, 0xC6, 0x61, 0x0C, 0xA0, 0xFE, 0x75, 0xDE, 0x66, 0xAE, 0xAB, 0x88, 0xB0, 0xB8, 0xBD,
    0xE6, 0x59, 0x67, 0xD1, 0x94, 0xBE, 0xA0, 0xB8, 0xBC, 0xF4, 0x7A, 0x9A, 0x40, 0x0F, 0x42, 0xC0,
    0x61, 0x83, 0xE7, 0xE0, 0x28, 0x4E, 0xE7, 0x16, 0xFE, 0x7A, 0xCF, 0xB7, 0x0D, 0x9E, 0x73, 0x5A,
    0x88, 0x3A, 0xD9, 0x8A, 0x53, 0x69, 0x62, 0xD7, 0x1D, 0x63, 0x24, 0xE6, 0x31, 0x9A, 0xC3, 0xD0,
    0xDB, 0xD8, 0x86, 0x99, 0x80, 0x4E, 0xD1, 0x5D, 0x6E, 0x76, 0x81, 0xDE, 0xA4, 0xC0, 0xD8, 0x87,
    0xDC, 0x7A, 0xBA, 0xDE, 0xAD, 0xBC, 0xC1, 0x73, 0x50, 0xA2, 0x9C, 0x1E, 0xA6, 0x31, 0xE7, 0x7D,
    0x9D, 0x71, 0x8A, 0x00, 0x53, 0xD6, 0x8E, 0x6D, 0xEB, 0x89, 0xA6, 0x86, 0xCC, 0x76, 0xED, 0x42,
    0x46, 0x20, 0xEE, 0xF3, 0xD1, 0xF3, 0xFA, 0xB6, 0x7B, 0xC9, 0xCE, 0x99, 0xF9, 0xF0, 0x80, 0xC5,
    0x60, 0x1E, 0xF3, 0x80, 0xC5, 0x4E, 0x66, 0xC0, 0xB8, 0x4A, 0xE7, 0x72, 0x73, 0xB8, 0x0F, 0x39,
    0x07, 0x3B, 0xE3, 0xB9, 0x26, 0x6D, 0xF3, 0xDA, 0x29, 0xF9, 0xAE, 0xA7, 0x48, 0xFA, 0xF0, 0x6B,
    0xFD, 0x02, 0x63, 0xE0, 0x5D, 0x4E, 0x5A, 0x91, 0x88, 0xED, 0xE7, 0xAF, 0xEA, 0x4E, 0x21, 0xEB,
    0xC6, 0x0A, 0x2A, 0x0A, 0xFA, 0xAD, 0xFC, 0x3C, 0x64, 0xE3, 0xFB, 0xAE, 0xC1, 0xFA, 0x2A, 0xA4,
    0xED, 0x6A, 0xD1, 0xDB, 0x65, 0xB6, 0x6D, 0x58, 0x19, 0x54, 0xF7, 0xF4, 0xD4, 0xA2, 0xF6, 0x82,
    0x29, 0x36, 0xF5, 0xEA, 0x39, 0xA6, 0x31, 0xF3, 0xA1, 0x28, 0xE3, 0xAC, 0x12, 0xF0, 0xA5, 0x29,
    0x33, 0xCA, 0x36, 0x23, 0x50, 0x51, 0xEF, 0xEC, 0x4F, 0xE5, 0x69, 0x53, 0xED, 0xE8, 0xC8, 0x57,
    0xFA, 0x65, 0x9C, 0xEB, 0x62, 0xD2, 0xBF, 0x71, 0x7A, 0x16, 0x9C, 0xE7, 0x13, 0x8A, 0x32, 0x4E,
    0x39, 0x9E, 0xD3, 0x8D, 0x74, 0x05, 0x8C, 0x9E, 0x36, 0x90, 0x03, 0x54, 0xDF, 0xDC, 0xD4, 0xA3,
    0xDE, 0x42, 0x41, 0xD1, 0x99, 0xDD, 0xD8, 0x1C, 0x65, 0xD0, 0xB8, 0x1E, 0xDB, 0x26, 0xD4, 0x44,
    0x26, 0x25, 0x82, 0x58, 0x64, 0xE7, 0xB4, 0xD7, 0xD0, 0x2E, 0x18, 0xFD, 0x51, 0x8A, 0xE7, 0x21,
    0xEA, 0x1E, 0x40, 0x1B, 0xCF, 0x82, 0x19, 0xA7, 0x1C, 0x8F, 0x80, 0xEB, 0xC4, 0x68, 0xD2, 0xC0,
    0x7D, 0xE1, 0xF9, 0xFE, 0xE0, 0x3C, 0x81, 0xA4, 0xBE, 0xBC, 0xE9, 0x50, 0xB8, 0x8B, 0xBD, 0xB8,
    0xEC, 0xC8, 0x20, 0xE1, 0x8A, 0x56, 0xFE, 0x39, 0x6F, 0x4E, 0x0C, 0xA3, 0x83, 0x74, 0xB7, 0xB0,
    0x8E, 0x32, 0x0E, 0x97, 0xE9, 0x89, 0x5C, 0x8F, 0x5D, 0x2C, 0xAF, 0x08, 0xA0, 0x0F, 0x4A, 0x36,
    0xBE, 0xD5, 0x9F, 0x81, 0x90, 0x1E, 0xD1, 0x7B, 0xF0, 0xE5, 0xFE, 0x83, 0xF3, 0xA2, 0x7E, 0x80,
    0x7F, 0x21, 0x70, 0x7E, 0x7C, 0xE9, 0x51, 0xA9, 0xE2, 0x7D, 0xA9, 0x78, 0x60, 0xD0, 0x53, 0x52,
    0x9A, 0xDB, 0xFE, 0xDC, 0x79, 0xD1, 0xEB, 0x6B, 0x4E, 0x32, 0x77, 0x04, 0x70, 0x35, 0x4E, 0x39,
    0xB2, 0xFE, 0xEE, 0x7C, 0x0B, 0xA0, 0x50, 0x60, 0x95, 0x63, 0x9B, 0x1C, 0x5F, 0x58, 0x38, 0xF1,
    0xD2, 0xFE, 0xE5, 0xFC, 0x3D, 0x3C, 0x30, 0x60, 0xFC, 0x08, 0x7E, 0x60, 0x06, 0x7E, 0x80, 0xFD,
    0x28, 0x0F, 0x7E, 0x40, 0xEF, 0x30, 0x80, 0xDF, 0x13, 0x7C, 0x3F, 0x3F, 0x7E, 0x50, 0xDF, 0x30,
    0x40, 0x6F, 0x01, 0xC0, 0x7E, 0x01, 0xA0, 0xDF, 0xD9, 0x02, 0x3C, 0x3E, 0xA6, 0x3F, 0x69, 0x7F,
    0x0B, 0xFD, 0x03, 0x08, 0x1F, 0x05, 0xEC, 0x03, 0x00, 0x28, 0x20, 0xAD, 0x72, 0x6E, 0x75, 0x8F,
    0xD8, 0x02, 0xFB, 0x03, 0x18, 0x08, 0x8D, 0x2F, 0xF5, 0x08, 0x10, 0xDB, 0x8C, 0x9F, 0x19, 0x78,
    0xE0, 0x74, 0xB0, 0x41, 0x63, 0x2F, 0xAB, 0x03, 0x1E, 0xC5, 0x41, 0x07, 0x00, 0x12, 0x88, 0x80,
    0x81, 0xFE, 0x69, 0x23, 0x40, 0x1B, 0x27, 0x10, 0x7D, 0x17, 0xF1, 0x22, 0x0D, 0xE0, 0x02, 0x11,
    0xF0, 0x73, 0x14, 0x6B, 0x7D, 0x10, 0x2B, 0x7E, 0x3E, 0xC4, 0x00, 0x7F, 0x07, 0x7E, 0xF0, 0xF8,
    0x00, 0x07, 0x1F, 0x7E, 0x34, 0xC8, 0x17, 0x03, 0xD0, 0x33, 0x3D, 0x01, 0x7E, 0x08, 0xFA, 0x03,
    0x17, 0xA1, 0x07, 0x3B, 0xA1, 0xBE, 0x78, 0x3D, 0xE8, 0xC4, 0x3E, 0xD1, 0x63, 0xAC, 0x0C, 0x1C,
    0x0E, 0x1E, 0x05, 0x3F, 0x7E, 0x55, 0xE4, 0x0F, 0x0F, 0x7E, 0x78, 0x7E, 0xFE, 0x20, 0xF5, 0x0B,
    0x83, 0xBE, 0x78, 0x0A, 0xE8, 0xC4, 0x5E, 0xD1, 0x63, 0x94, 0x0B, 0x0C, 0x05, 0x5F, 0xFE, 0x15,
    0xE4, 0x07, 0x47, 0x42, 0x0F, 0x42, 0x43, 0x7D, 0xF1, 0x65, 0xD0, 0x89, 0x6E, 0xA3, 0xC7, 0x58,
    0x41, 0xA1, 0x40, 0x6F, 0xAE, 0xD0, 0x83, 0x63, 0x50, 0xD8, 0xB0, 0x61, 0xB0, 0x8E, 0xEA, 0x62,
    0x52, 0xD1, 0x4C, 0x65, 0x70, 0x8A, 0xB2, 0x60, 0x50, 0x50, 0x77, 0x8E, 0xD5, 0x0A, 0x53, 0x8A,
    0x45, 0x1C, 0x79, 0xA2, 0x56, 0x35, 0xA6, 0xEA, 0x78, 0x40, 0xB9, 0x53, 0x26, 0x21, 0x78, 0x70,
    0x79, 0x7B, 0x7B, 0x0A, 0x8E, 0xA7, 0x9E, 0x29, 0x92, 0x2B, 0xD5, 0x4C, 0x56, 0x1A, 0x7D, 0x70,
    0x32, 0x8A, 0x9C, 0x9A, 0x98, 0xA0, 0x84, 0x54, 0x7E, 0x25, 0x20, 0x7F, 0x7E, 0x4D, 0x7F, 0x03,
    0xA0, 0x6F, 0xFC, 0x8F, 0x7E, 0xC0, 0x73, 0x03, 0x1F, 0x87, 0x8F, 0xD0, 0x83, 0x83, 0x0F, 0xD0,
    0xE3, 0x8D, 0x0E, 0xE1, 0x71, 0x8E, 0x46, 0x3C, 0x8F, 0x26, 0x54, 0x80, 0xA8, 0xA0, 0x9F, 0xFE,
    0xAE, 0x75, 0x03, 0xBE, 0x18, 0x93, 0x6D, 0x5E, 0x86, 0xBA, 0xC0, 0x33, 0x83, 0x8F, 0xBE, 0x28,
    0x18, 0x76, 0xBE, 0xA9, 0x84, 0x10, 0xCA, 0x7F, 0x94, 0xDD, 0x80, 0x8F, 0x1B, 0xAF, 0xEE, 0xF4,
    0xB4, 0x83, 0x20, 0xD6, 0x6E, 0x71, 0x30, 0x83, 0x5E, 0x3C, 0x17, 0x58, 0x39, 0x66, 0xA3, 0x31,
    0xB7, 0x85, 0x0E, 0xB1, 0xA1, 0xD0, 0xB0, 0xF2, 0x98, 0x49, 0x88, 0xB0, 0x4D, 0x57, 0xB9, 0xBB,
    0x86, 0xA3, 0x8A, 0x68, 0xCA, 0x80, 0x54, 0x20, 0xD1, 0x8E, 0x84, 0x92, 0x75, 0xBD, 0xA0, 0x35,
    0x86, 0x18, 0x6A, 0xA2, 0xA9, 0x82, 0x2A, 0x94, 0x40, 0xA1, 0xBE, 0x82, 0xBF, 0xD2, 0x60, 0xC0,
    0xA5, 0xAA, 0x9C, 0xD3, 0x09, 0x81, 0x42, 0x0F, 0x42, 0xC7, 0x81, 0x07, 0x92, 0x6F, 0xE7, 0x18,
    0x05, 0xD7, 0xC1, 0x9C, 0xB6, 0x14, 0xA0, 0x98, 0xFA, 0x65, 0xC7, 0xC0, 0xCF, 0xAF, 0xD0, 0xC1,
    0x73, 0xE0, 0x54, 0x7D, 0x8C, 0x91, 0xC3, 0x13, 0x4A, 0x96, 0x74, 0x42, 0x80, 0x64, 0x72, 0x38,
    0x71, 0xC1, 0xC7, 0xD7, 0x38, 0xB4, 0xC0, 0x0A, 0xCA, 0x94, 0xA9, 0xD8, 0xDB, 0x32, 0x34, 0xAC,
    0xA0, 0x4C, 0x29, 0x16, 0x69, 0xDD, 0x82, 0xD8, 0xD3, 0x4C, 0x82, 0xE7, 0x19, 0x82, 0x77, 0x12,
    0x28, 0xDE, 0xD4, 0xDF, 0x25, 0xF4, 0x20, 0xE3, 0x14, 0x1E, 0xAE, 0x38, 0xE1, 0x3A, 0x2D, 0xC0,
    0x44, 0x35, 0xE1, 0xE3, 0xE7, 0x0E, 0x25, 0x52, 0x10, 0x50, 0x17, 0x73, 0xB1, 0xEB, 0x02, 0xE0,
    0xC9, 0x41, 0x9D, 0xED, 0x0A, 0x65, 0xCA, 0xC8, 0x22, 0xE4, 0x01, 0xCE, 0x04, 0x3B, 0xEC, 0xDD,
    0xEF, 0x25, 0x74, 0xB0, 0x82, 0x12, 0x29, 0x04, 0x39, 0x0F, 0xF3, 0x47, 0xCD, 0xC8, 0xD4, 0xF5,
    0x21, 0x2C, 0x52, 0x55, 0xC2, 0x99, 0x65, 0xC0, 0xCA, 0x2C, 0xBE, 0xE6, 0x21, 0x05, 0x4E, 0xAE,
    0x39, 0x93, 0xF9, 0xAA, 0xA3, 0xF9, 0x80, 0x12, 0x59, 0xFA, 0xA5, 0xFB, 0x2F, 0x54, 0xF8, 0xA8,
    0x00, 0x61, 0x90, 0xCA, 0x2C, 0xB1, 0xE6, 0xC1, 0x86, 0x31, 0x2B, 0xC0, 0x7F, 0x81, 0xF6, 0x15,
    0x1C, 0xE0, 0x3C, 0x68, 0x3F, 0x0A, 0xFB, 0x80, 0x0C, 0x5E, 0x5F, 0xE9, 0x50, 0x07, 0x4C, 0x91,
    0x5D, 0x6F, 0x2E, 0x74, 0x3F, 0x38, 0x1F, 0x1B, 0xE7, 0x91, 0x3A, 0x62, 0x77, 0x00, 0xBE, 0x60,
    0xDB, 0x26, 0xF3, 0x68, 0xAE, 0x6A, 0xE3, 0x7B, 0xC3, 0x9A, 0x24, 0x70, 0x09, 0x9B, 0x66, 0x1A,
    0x28, 0x7C, 0xD4, 0xFF, 0x7A, 0x7F, 0xA1, 0x0F, 0x7F, 0x94, 0x3E, 0x58, 0x32, 0xC7, 0x2C, 0x48,
    0xCB, 0x81, 0x02, 0x6D, 0x8E, 0x9F, 0xC3, 0xDF, 0xA4, 0x0A, 0xA6, 0x3C, 0x04, 0x18, 0x63, 0x64,
    0x7F, 0x19, 0xE9, 0x40, 0x12, 0xB7, 0x12, 0x1A, 0xDF, 0xE2, 0xC9, 0x94, 0x85, 0xBB, 0x05, 0x22,
    0xB9, 0x89, 0x17, 0x35, 0x21, 0xB0, 0x2C, 0xB8, 0xB0, 0x8F, 0x22, 0x09, 0xBE, 0x75, 0xBF, 0x05,
    0x1E, 0x28, 0x19, 0x54, 0xD0, 0x58, 0xC6, 0x38, 0xD9, 0x28, 0x1B, 0xCF, 0xB8, 0x86, 0x09, 0x98,
    0xFF, 0x01, 0xD8, 0x41, 0xC3, 0xD9, 0x42, 0x09, 0xC0, 0x15, 0x6C, 0x42, 0x2C, 0xD0, 0x05, 0xD8,
    0x8F, 0x15, 0x24, 0xA1, 0xDE, 0xDF, 0xAE, 0x90, 0x21, 0x6A, 0x73, 0x50, 0x2F, 0xC0, 0xFF, 0x3B,
    0x15, 0xBF, 0x1C, 0x86, 0x2A, 0x8F, 0x95, 0x9D, 0x4D, 0xEC, 0xAD, 0xEE, 0xE9, 0xEF, 0x0A, 0x61,
    0x8B, 0x2B, 0xB0, 0x17, 0xE1, 0x14, 0xF4, 0x69, 0xF6, 0xBA, 0xF7, 0x42, 0x85, 0xF0, 0xEA, 0x59,
    0x85, 0x0F, 0x2B, 0x30, 0xF8, 0x16, 0x14, 0xA1, 0xAE, 0xFE, 0xD9, 0xFF, 0x11, 0xFE, 0x94, 0xD3,
    0xFD, 0x0A, 0xFC, 0x8A, 0xF8, 0xFB, 0xD7, 0xFA, 0x14, 0x50, 0xF8, 0xC8, 0x21, 0xF7, 0x6B, 0xF6,
    0x0A, 0x45, 0x29, 0xF4, 0x4E, 0xD8, 0x70, 0x18, 0xF0, 0xA8, 0x4D, 0x8C, 0xEF, 0x15, 0x9A, 0x12,
    0xEE, 0xB5, 0x0A, 0x3A, 0xEC, 0xDC, 0x0E, 0xE8, 0x47, 0xE8, 0x10, 0xE7, 0x0A, 0x13, 0x0A, 0xC3,
    0x1C, 0x50, 0xE0, 0x7D, 0x71, 0x80, 0x0A, 0xDF, 0xB5, 0x60, 0xDE, 0xA1, 0x54, 0x87, 0xDC, 0x9B,
    0xD8, 0x82, 0x45, 0x61, 0xD9, 0x9C, 0xD0, 0xA0, 0x4C, 0x38, 0xD7, 0x04, 0x1A, 0x44, 0x86, 0x41,
    0xCF, 0x04, 0x0C, 0x91, 0x86, 0x00, 0x29, 0x0C, 0xE9, 0xC2, 0xC0, 0x7A, 0x07, 0x80, 0xA1, 0x42,
    0xBF, 0x2D, 0xBE, 0x58, 0x08, 0x31, 0x83, 0xBC, 0x85, 0x42, 0xB8, 0x85, 0xCB, 0x30, 0xBB, 0xA1,
    0xA8, 0xA5, 0x43, 0x3A, 0x42, 0xB7, 0x8F, 0xB0, 0xA0, 0x57, 0xC8, 0x41, 0xAF, 0x28, 0xE0, 0x06,
    0xE0, 0x4A, 0x63, 0x0F, 0xCF, 0x6D, 0x16, 0x50, 0x68, 0x1E, 0x9F, 0x9E, 0x88, 0x0E, 0x18, 0x94,
    0x83, 0x9B, 0x14, 0x85, 0xD0, 0x03, 0x2A, 0xD2, 0x85, 0x07, 0x80, 0xF7, 0x00, 0x81, 0x7F, 0x2F,
    0x14, 0x7E, 0x2A, 0x14, 0x19, 0x7C, 0x48, 0xB0, 0xCD, 0x2D, 0xA6, 0x21, 0x7B, 0x78, 0x43, 0x19,
    0x77, 0x64, 0x26, 0x20, 0x03, 0x13, 0xE5, 0x77, 0xB9, 0x70, 0xC1, 0x0A, 0xC3, 0x65, 0x08, 0x81,
    0xE2, 0x34, 0x4C, 0x7D, 0x6F, 0x64, 0x60, 0x38, 0x64, 0x40, 0xB8, 0x50, 0x07, 0x5F, 0xA6, 0x5E,
    0x4C, 0xC3, 0x3E, 0x60, 0x82, 0x3F, 0x3E, 0x9E, 0x0B, 0xF2, 0x01, 0xEF, 0x05, 0xFE, 0x7E, 0x00,
    0xFD, 0x73, 0x00, 0xEF, 0xFE, 0x35, 0x1F, 0x7E, 0x40, 0xDF, 0x31, 0x40, 0x38, 0x47, 0x28, 0x3C,
    0x3E, 0x03, 0x7F, 0xBE, 0x73, 0x00, 0x80, 0x7E, 0x62, 0x0F, 0x40, 0x0F, 0x7E, 0xEF, 0xC1, 0x00,
    0x1C, 0xCD, 0x1E, 0xB4, 0x1F, 0x05, 0x3F, 0x7E, 0x3D, 0x5F, 0xC0, 0x2F, 0x1E, 0x07, 0xB4, 0xC0,
    0x07, 0xA4, 0x03, 0x14, 0x7A, 0x10, 0x01, 0xEA, 0xEB, 0x06, 0x0E, 0x6D, 0x0F, 0x9A, 0x1F, 0x82,
    0x1E, 0x88, 0x46, 0x00, 0x83, 0xD3, 0x7A, 0x80, 0x20, 0x87, 0x8F, 0xD4, 0x4E, 0xC7, 0xC3, 0x58,
    0x0E, 0x8C, 0x61, 0xD0, 0x62, 0x42, 0xC0, 0xED, 0x04, 0xE0, 0xE3, 0xA6, 0xD0, 0xF0, 0x83, 0xF1,
    0x7B, 0xF8, 0xD8, 0xFC, 0xB1, 0xFE, 0x09, 0xFF, 0x06, 0x00, 0x00, 0x71, 0x28, 0xBC, 0x58, 0x01,
    0x04, 0x00, 0x60, 0x7D, 0x6E, 0xAB, 0x2A, 0x58, 0xD1, 0x88, 0x00, 0x00, 0x00, 0xA0, 0x26, 0x9C,
    0x15, 0x8D, 0x44
};