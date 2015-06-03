//
// SourceDest.cpp
//
// Written by Johannes Hjorth, based on original KeyValue.cpp
// by Matthew Flood.
//

#include "../config.h"

#include "SourceDest.h"

#ifndef INCLUDED_AbstractWriter_H
#include "AbstractWriter.h"
#endif

#ifndef INCLUDED_CSTDIO
#include <cstdio>
#define INCLUDED_CSTDIO
#endif

using namespace std;
namespace rude{
namespace config{

SourceDest::SourceDest()
{
	d_srcApp = "";
	d_srcObj = "";
	d_destApp = "";
	d_destObj = "";
	d_width = "";
	d_comment = "";
	d_commType = "";
	d_procMethod = "";
}

SourceDest::SourceDest(const char *srcApp, const char *srcObj, const char *destApp,  const char *destObj,
		const char *width,const char* commType, const char *procMethod, const char *comment)
{
	d_srcApp  = srcApp  ? srcApp  : "";
	d_srcObj  = srcObj  ? srcObj  : "";
	d_destApp = destApp ? destApp : "";
	d_destObj = destObj ? destObj : "";
	d_width   = width   ? width   : "";
	d_comment = comment ? comment : "";
	d_commType = commType ? commType : "";
	d_procMethod = procMethod ? procMethod : "";
}

void SourceDest::acceptWriter(AbstractWriter& writer) const
{
  std::cout << "writer.visitSourceDest not implemented yet!";
  std::cout << "\nsrcApp: \"" << d_srcApp << "\", srcObj: \"" << d_srcObj << "\"";
  std::cout << "\ndestApp: \"" << d_srcApp << "\", destObj: \"" << d_srcObj << "\"";
  std::cout << "\nwidth: " << d_width;
  std::cout << "\ncommType: " << d_commType;
  std::cout << "\nprocMethod: " << d_procMethod;
  std::cout << "\ncomment: \"" << d_comment << "\"\n";
  //writer.visitKeyValue(*this);
}

std::string SourceDest::toString()
{
  std::cout << "REMOVE THIS FUNCTION";

  return   "\nsrcApp: \"" + d_srcApp + "\", srcObj: \"" + d_srcObj + "\"" 
           + "\ndestApp: \"" + d_destApp + "\", destObj: \"" + d_destObj + "\""
           + "\nwidth: " + d_width
           + "\ncommType: " + d_commType
           + "\nprocMethod: " + d_procMethod
           + "\ncomment: \"" + d_comment + "\"\n";
}


const char *SourceDest::getSrcApp() const
{
	return d_srcApp.c_str();
}

const char *SourceDest::getSrcObj() const
{
	return d_srcObj.c_str();
}

const char *SourceDest::getDestApp() const
{
	return d_destApp.c_str();
}

const char *SourceDest::getDestObj() const
{
	return d_destObj.c_str();
}

const char *SourceDest::getWidth() const
{
	return d_width.c_str();
}
const char *SourceDest::getCommType() const
{
	return d_commType.c_str();
}
const char *SourceDest::getProcMethod() const
{
	return d_procMethod.c_str();
}
const char *SourceDest::getComment() const
{
	return d_comment.c_str();
}

void SourceDest::setSrcApp(const char *srcApp)
{
	d_srcApp = srcApp ? srcApp : "";
}

void SourceDest::setSrcObj(const char *srcObj)
{
	d_srcObj = srcObj ? srcObj : "";
}

void SourceDest::setDestApp(const char *destApp)
{
	d_destApp = destApp ? destApp : "";
}

void SourceDest::setDestObj(const char *destObj)
{
	d_destObj = destObj ? destObj : "";
}

void SourceDest::setWidth(const char *width)
{
	d_width = width ? width : "";
}
void SourceDest::setCommType(const char *commType)
{
	d_commType = commType ? commType : "";
}
void SourceDest::setProcMethod(const char *procMethod)
{
	d_procMethod = procMethod ? procMethod : "";
}
void SourceDest::setComment(const char *comment)
{
	d_comment = comment ? comment : "";
}

SourceDest::~SourceDest()
{

}
}} // end namespace rude::config

