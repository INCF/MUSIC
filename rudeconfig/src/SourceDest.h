// SourceDest.h
//
// Added by Johannes Hjorth, 2008
// 
// Feel free to use the code, have fun!
//
// Based on the KeyValue-code




#ifndef INCLUDED_SourceDest_H
#define INCLUDED_SourceDest_H

#ifndef INCLUDED_DataLine_H
#include "DataLine.h"
#endif

#ifndef INCLUDED_STRING
#include <string>
#define INCLUDED_STRING
#endif

#ifndef INCLUDED_IOSTREAM
#include <iostream>
#define INCLUDED_IOSTREAM
#endif

namespace rude{
namespace config{
//=
// SourceDest represents message passing from Source to Dest
// 
// app1.source -> dest
// dest <- app2.source
// app2.dest <- source
//

class SourceDest: public DataLine{


	std::string d_srcApp;
	std::string d_srcObj;
	std::string d_destApp;
	std::string d_destObj;
	std::string d_width;
	std::string d_comment;
	/* remedius
	 * <collective> or <point-to-point>
	 */
	std::string d_commType;
	/* remedius
	 * <tree> or <table>
	*/
	std::string d_procMethod;

public:

	// default constructor
	SourceDest();
	/* remedius
	 * two more parameters: commType and procMethod were added due to the runtime opportunity
	 * of choosing communication type and pre-/post-processing method.
	 */
	SourceDest(const char *srcApp, const char *srcObj, const char *destApp,  const char *destObj,
			const char *width, const char* commType, const char *procMethod, const char *comment);
	
	void acceptWriter(AbstractWriter& writer) const;
        std::string toString();

        //
	const char *getSrcApp() const;
	const char *getSrcObj() const;
	const char *getDestApp() const;
	const char *getDestObj() const;
	const char *getWidth() const;
    /* remedius
     *
     */
	const char *getCommType() const;
	/* remedius
	 *
	 */
	const char *getProcMethod() const;

	//= 
	// Returns the comment associated with the data member
	// Will return the comment even if the data member is flagged as being deleted
	// Always returns at least the empty string, will never return null.
	//=
	const char *getComment() const;

	//= 
	// Sets the name of the data member
	// Will set the name even if the data member is flagged as being deleted or a comment
	// Accepts null
	//=
	void setSrcApp(const char *name);
	void setSrcObj(const char *name);
	void setDestApp(const char *name);
	void setDestObj(const char *name);
	void setWidth(const char *name);
	/* remedius
	 *
	 */
	void setCommType(const char *commType);
	/* remedius
	 *
	 */
	void setProcMethod(const char *procMethod);

	//= 
	// Sets the comment associated with the data member
	// Will set the comment even if the data member is flagged as being deleted
	// Accepts null
	//=
	void setComment(const char *comment);
	
	~SourceDest();
	
};

}} // end namespace rude::config

#endif

