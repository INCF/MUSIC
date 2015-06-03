// RealOrganiser.h
//
// Copyright (C) 2000, 2001, 2002, 2003, 2004, 2005 Matthew Flood
// See file AUTHORS for contact information
//
// This file is part of RudeConfig.
//
// RudeConfig is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2, or (at your option)
// any later version.
// 
// RudeConfig is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with RudeConfig; (see COPYING) if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//------------------------------------------------------------------------

#ifndef INPUT_RealOrganiser_h
#define INPUT_RealOrganiser_h

#include "AbstractOrganiser.h"

#include <string>

namespace rude{
namespace config{

class File;
class Section;

class RealOrganiser: public AbstractOrganiser
{
	File *d_file;
	Section *d_section;
	
public:

	RealOrganiser(File*);

	virtual void foundSection(const char *sectionName, const char *comment);
	virtual void foundComment(const char *comment);
	virtual void foundWhiteSpace(const char *whitespace);
	virtual void foundData(const char *key, const char *value, const char *comment);
	/* remedius
	 * two more parameters: commType and procMethod were added due to the runtime opportunity
     * of choosing communication type and pre-/post-processing method.
     */
	virtual void foundSourceDest(const char *srcApp, const char *srcObj, const char *destApp, const char *destObj,
			const char *width, const char *commType, const char *procMethod, const char *comment);
	

};

}} // end namespaces

#endif

