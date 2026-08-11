// $Id: Array.cc 201 2008-05-18 19:47:38Z digasper $
// This file is part of QuadProg++:
// Copyright (C) 2006--2009 Luca Di Gaspero.
//
// This software may be modified and distributed under the terms
// of the MIT license.  See the LICENSE file for details.

// QuadProg++ 容器的索引辅助函数；与机器人算法无关，保留第三方实现便于求解器复用。

#include "Array.hh"

/**
  Index utilities
 */

std::set < unsigned int > seq(unsigned int s, unsigned int e)
{
  std::set < unsigned int > tmp;
  for (unsigned int i = s; i <= e; i++) {
    tmp.insert(i);
  }

  return tmp;
}

std::set < unsigned int > singleton(unsigned int i)
{
  std::set < unsigned int > tmp;
  tmp.insert(i);

  return tmp;
}
