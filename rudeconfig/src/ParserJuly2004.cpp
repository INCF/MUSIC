// ParserJuly2004.cc
//
// Copyright (C) 2000, 2001, 2002, 2003, 2004, 2005, 2007 Matthew Flood
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

#include "../config.h"

#include "ParserJuly2004.h"

#ifndef INCLUDED_AbstractOrganiser_h
#include "AbstractOrganiser.h"
#endif

#ifndef INCLUDED_CCTYPE
#include <cctype>
#define INCLUDED_CCTYPE
#endif

#ifndef INCLUDED_IOSTREAM
#include <iostream>
#define INCLUDED_IOSTREAM
#endif

#include <cstdio>

using namespace rude::config;
using namespace std;

namespace rude{
namespace config{

enum SectionState{ STARTSECTION, SECTIONID, ESCAPEID, ENDSECTIONID, SECTIONCOMMENT, FOUNDIDONLY, FOUNDIDCOMMENT, SECTIONERROR, ENDSECTION };
/* remedius
 * GETCOMMTYPE, GETPROCMETHOD were added in order to support runtime configuration options for choosing communication type and pre-/post-processing method
 */
enum KeyValueState{ KEY, KEYESCAPE, STARTVALUE, COMMENT, FINDCOMMENT, KVERROR, ENDKV, VALUE, QUOTEVALUE, NONQUOTEVALUE, QUOTEESCAPE, NONQUOTEESCAPE, ENDKEYVALUE, LEFTARROW, RIGHTARROW, GETSOURCE, GETDEST, GETWIDTH, GETCOMMTYPE, GETPROCMETHOD, SDCOMMENT, ENDSOURCEDEST};

void ParserJuly2004::stripTrailing(std::string& buffer)
{
	int bufferLength = buffer.size();

	for (int x = bufferLength - 1; x >= 0; x--)
	{
		char c = buffer[x];

		if (isspace(c))
		{
			buffer.erase(x);
		}
		else
		{
			break;
		}
	}
}


bool ParserJuly2004::isEOL(char c)
{
	return (c == '\r' || c == '\f' || c == '\n');
}

bool ParserJuly2004::chompEOL(std::istream& inputstream)
{
		char c = inputstream.peek();
		if(isEOL(c))
		{
			inputstream.get();
			char next_c = inputstream.peek();
			if( (c != next_c) && isEOL(next_c) )
			{
				inputstream.get();
			}
		}
		return true;
}

bool ParserJuly2004::parse(std::istream& infile, AbstractOrganiser& organiser)
{
	if (d_delimiter == '\\' || isEOL(d_delimiter) || d_delimiter == d_commentchar || d_delimiter == '[')
	{
		setError("110", "Illegal delimiter.");
		return false;
	}

	if (d_commentchar == '\\' || d_commentchar == '"' || isspace(d_commentchar))
	{
		setError("111", "Illegal comment character.");
		return false;
	}
	
	register int c;
		
	// eof only gets set when error_flag is set on previous operation
	// as such, you need to peek() at the end ot the while loop
	// in order for eof to happen when you want it to!!
	// one peek() triggers infile.eof(), but it does not reveal it!!!!
	// you gotta call peek() twice.
	//
	while( (c = infile.peek()) != EOF)
	{
		// We're looking for:
		// '[' beggining of section
		// '#' (d_commentchar) comment character
		// any non-whitespace character

		if(isspace(c))
		{
			std::string whitespace = "";

			while(c != EOF && isspace(c))
			{
				whitespace += infile.get();
				c = infile.peek();
			}
			organiser.foundWhiteSpace(whitespace.c_str());
		}
		else if(c == '[')
		{
			// discard '[' character
			//
			infile.get();
				
			register SectionState sectionState = STARTSECTION;

			std::string sectionID = "";
			std::string comment = "";
				
			while (sectionState != ENDSECTION)
			{						
				switch (sectionState)
				{
					case STARTSECTION:
					{
							c = infile.peek();
							if(c == EOF)
							{
								setError("102", "End of stream found before section ID.");
								sectionState = SECTIONERROR;
							}
							else if(isEOL(c))
							{
								setError("101", "End of line found before section ID.");
								sectionState = SECTIONERROR;   
							}
							else if(c == ' ' || c == '\t')
							{
								// discard whitespace
								//
								infile.get();
									
								// LOOP
							}
							else if(c == ']')
							{
								// discard ']'
								//
								infile.get();
								
								sectionState = ENDSECTIONID;
							}
							else
							{
								sectionState = SECTIONID;
							}
							break;
					}
					case SECTIONID:
					{
							c = infile.peek();
							if(c == EOF)
							{
								setError("104", "End of stream found before end-of-section marker.");
								sectionState = SECTIONERROR;
							}
							else if(isEOL(c))
							{
								setError("103", "End of line found before end-of-section marker.");
								sectionState = SECTIONERROR;	     
							}
							else if(c == '\\')
							{
								// discard backslash
								//
								infile.get();

								sectionState = ESCAPEID;
							}
							else if(c == ']')
							{
								// discard ']'
								//
								infile.get();

								// Strip Trailing Whitespace from ID
								//
								stripTrailing(sectionID);
									
								sectionState = ENDSECTIONID;
							}
							else
							{
								sectionID += infile.get();

								// LOOP
							}
							break;
					}
					case ESCAPEID:
					{
							c = infile.peek();
							if(c == EOF)
							{
								setError("107", "End of stream found after un-escaped backslash.");
								sectionState = SECTIONERROR;
							}
							else if(isEOL(c))
							{
								setError("108", "Escaped new-line is not allowed in section ID or key.");
								sectionState = SECTIONERROR;
							}
							else
							{
								sectionID += infile.get();
								sectionState = SECTIONID;
							}
							break;
					}
					case ENDSECTIONID:
					{
							c = infile.peek();
							if(c == EOF || isEOL(c))
							{
								sectionState = FOUNDIDONLY;				
							}
							else if(d_commentchar != 0 && c == d_commentchar)
							{
								// discard '#'
								//
								infile.get();
									
								sectionState = SECTIONCOMMENT;								
							}
							else if(c == ' ' || c == '\t')
							{
								// discard whitespace
								//
								infile.get();
									
								// LOOP
							}
							else
							{
								setError("105", "Illegal character found after end-of-section marker.");
								sectionState = SECTIONERROR;
							}
							break;
					}
					case SECTIONCOMMENT:
					{
							c = infile.peek();
							if(c == EOF || isEOL(c))
							{
								sectionState = FOUNDIDCOMMENT;
								stripTrailing(comment);
							}
							else
							{
								// append to comment
								//
								comment += infile.get();
									
								// LOOP			
							}
							break;
					}
					case SECTIONERROR:
					case ENDSECTION: // dummy
					{
						return false;
					}
					case FOUNDIDONLY:
					{
						organiser.foundSection(sectionID.c_str(), 0);
						chompEOL(infile);
						sectionState = ENDSECTION;
						break;
					}
					case FOUNDIDCOMMENT:
					{
						organiser.foundSection(sectionID.c_str(), comment.c_str());
						chompEOL(infile);
						sectionState = ENDSECTION;
						break;
					}
				}
			}
		}
		else if(c == d_commentchar)
		{
			// found a comment line
			// discard the comment character
			//
			infile.get();
				
			// put the rest of the line into a string
			//
			std::string line="";
			while(infile.good())
			{
				c=infile.get();
				if( isEOL(c))
				{
					break;
				}
				line += c;
			}

			chompEOL(infile);
			
			// PROCESS THE COMMENT LINE
			//
			stripTrailing(line);
			organiser.foundComment(line.c_str());
		}
		else
		{
			register KeyValueState kvState = KEY;
			std::string key = "";
			std::string value = "";
			std::string comment = "";


                        // Added to handle source -> dest

                        std::string srcApp = "";
                        std::string srcObj = "";
                        std::string destApp = "";
                        std::string destObj = "";
                        std::string width = "";
                        /* remedius */
                        std::string commType = "";
                        /* remedius */
                        std::string procMethod = "";

			while (kvState != ENDKEYVALUE)
			{						
				
				switch (kvState)
				{
					case KEY:
					{
                                          //std::cout << "KEY\n";

							int c = infile.peek();
							
							if(c == EOF || isEOL(c))
							{
								kvState = ENDKV;
							}
							else if(d_delimiter && c == d_delimiter)
							{
								// discard '='
								//
								infile.get();

								kvState = STARTVALUE;
							}
							else if(!d_delimiter && (c == ' ' || c == '\t'))
							{
								// discard whitespace
								//
								infile.get();
								kvState = STARTVALUE;
							}
							else if(d_commentchar != 0 && c == d_commentchar)
							{
								// discard '#'
								//
								infile.get();

								kvState = COMMENT;
							}
							else if(c == '\\')
							{
								// discard '\\'
								//
								infile.get();

								kvState = KEYESCAPE;
							}
                                                        else if(c == '-')
                                                          {
                                                            // Is it -> ??
                                                            infile.get();
                                                            c = infile.peek();

                                                            if(c == '>')
                                                              {
                                                                infile.get();

                                                                kvState = RIGHTARROW;
                                                                // We need to split up the key into srcApp.srcObj and store them
                                                                size_t dotPos = key.find_first_of('.');

                                                                if(dotPos != string::npos)
                                                                  {
                                                                    srcApp = key.substr(0,dotPos);
                                                                    srcObj = key.substr(dotPos+1,key.length()-dotPos-1);

                                                                    //std::cout << "Found srcApp: " << srcApp << " srcObj: " << srcObj << "\n";

                                                                  }
                                                                else
                                                                  {
                                                                    srcApp = "";
                                                                    srcObj = key;
                                                                  }
                                                              }
                                                            else
                                                              {
                                                                key += '-';
                                                              }
                                                          }
                                                        else if(c == '<')
                                                          {
                                                            // IS it <- ??
                                                            infile.get();
                                                            c = infile.peek();

                                                            if(c == '-')
                                                              {
                                                                infile.get();

                                                                kvState = LEFTARROW;
                                                                // We need to split up the key into destApp.destObj and store them
                                                                size_t dotPos = key.find_first_of('.');

                                                                if(dotPos != string::npos)
                                                                  {

                                                                    destApp = key.substr(0,dotPos);
                                                                    destObj = key.substr(dotPos+1,key.length()-dotPos-1);

                                                                    //std::cout << "Found destApp: " << destApp << " destObj: " << destObj << "\n";

                                                                  }
                                                                else
                                                                  {
                                                                    destApp = "";
                                                                    destObj = key;
                                                                  }
                                                              }
                                                            else
                                                              {
                                                                key += '<';
                                                              }
                                                          }
							else
							{
								// append to key
								//
								key += infile.get();
									
								// LOOP			
							}
							break;
					}
					case KEYESCAPE:
					{
                                          //std::cout << "KEYESCAPE\n";
							int c = infile.peek();
							if(c == EOF)
							{
								setError("107", "End of stream found after un-escaped backslash.");
								kvState = KVERROR;
							}
							else if(isEOL(c))
							{
								setError("108", "Escaped new-line is not allowed in key.");
								kvState = KVERROR;
							}
							else
							{
								// append to key
								//
								key += infile.get();

								kvState = KEY;
							}
							break;
					}
					case STARTVALUE:
					{
                                          //std::cout << "STARTVALUE\n";
							int c = infile.peek();

							if(c == EOF || isEOL(c))
							{
								kvState = ENDKV;
							}
							else if(d_commentchar != 0 && c == d_commentchar)
							{
								// discard '#'
								//
								infile.get();

								kvState = COMMENT;
							}
							else if(c == ' ' || c == '\t')
							{
								// discard whitespace
								//
								infile.get();

								// LOOP
							}
							else
							{
								kvState = VALUE;	
							}
							break;
					}
					case KVERROR:
					case ENDKEYVALUE: // dummy for compiler
					{
                                          //std::cout << "KVERROR\n";
							return false;
					}
					case FINDCOMMENT:
					{
                                          
                                          //std::cout << "FINDCOMMENT\n";
							int c = infile.peek();

							if(c == EOF || isEOL(c))
							{
								kvState = ENDKV;
							}
							else if(d_commentchar != 0 && c == d_commentchar)
							{
								// discard '#'
								//
								infile.get();

								kvState = COMMENT;
							}
							else if(c == ' ' || c == '\t')
							{
								// discard whitespace
								//
								infile.get();

								// LOOP
							}
							else
							{
								setError("109", "Illegal Character Found after quoted value.");
								kvState = KVERROR;	
							}
							break;
					}
					case COMMENT:
					{
                                          //std::cout << "COMMENT\n";

							int c = infile.peek();

							if(c == EOF || isEOL(c))
							{
								stripTrailing(comment);
								kvState = ENDKV;
							}
							else
							{
								// Append to comment
								//
								comment += infile.get();

								// LOOP	
							}
							break;
					}
					case VALUE:
					{
                                          //std::cout << "VALUE\n";

						int c = infile.peek();
						if(c == '"')
						{
							// discard '"'
							//
							infile.get();

							kvState = QUOTEVALUE;
						}
						else
						{
							kvState = NONQUOTEVALUE;
						}
						break;
					}
					case QUOTEVALUE:
					{
                                          //std::cout << "QUOTEVALUE\n";
							int c = infile.peek();
							if(c == EOF)
							{
								setError("106", "End of stream found before final quote (\") in value.");
								kvState = KVERROR;
							}
							else if(c == '"')
							{
								// discard '"'
								//
								infile.get();

								kvState = FINDCOMMENT;
							}
							else if(c == '\\')
							{
								// discard backslash
								//
								infile.get();

								kvState = QUOTEESCAPE;
							}
							else
							{
								// append to value
								//
								value += infile.get();

								// LOOP
							}
							break;
					}
					case QUOTEESCAPE:
					{
                                          //std::cout << "QUOTEESCAPE\n";

							int c = infile.peek();
							if(c == EOF)
							{
								setError("107", "End of stream found after un-escaped backslash.");
								kvState = KVERROR;
							}
							else
							{
								// append to value
								//
								value += infile.get();

								kvState = QUOTEVALUE;
							}
							break;
					}
					case NONQUOTEVALUE:
					{
                                          //std::cout << "NONQUOTEVALUE\n";
							int c = infile.peek();

							if(c == EOF || isEOL(c))
							{
								stripTrailing(value);

								kvState = ENDKV;
							}
							else if(d_commentchar != 0 && c == d_commentchar)
							{
								// discard '#'
								//
								infile.get();
								
								stripTrailing(value);
								
								kvState = COMMENT;
							}
							else if(c == '\\')
							{
								// discard backslash
								//
								infile.get();

								kvState = NONQUOTEESCAPE;
							}
							else
							{
								// append to value
								//
								value += infile.get();

								// LOOP
							}
							break;
					}
					case NONQUOTEESCAPE:
					{
                                          //std::cout << "NONQUOTEESCAPE\n";

							int c = infile.peek();
							if(c == EOF)
							{
								setError("107", "End of stream found after un-escaped backslash.");
								kvState = KVERROR;
							}
							else
							{
								// append to value
								//
								c = infile.get();
								value += c;

								// SPECIAL CASE FOR ESCAPED CRLFs:
								//
								// if c is newline and next character is also newline, 
								// we keep both of them if they are different forms of newline
								// 
								char next_c = infile.peek();
								if(isEOL(c) && isEOL(next_c) && (c != next_c))
								{
									value += infile.get();
								} 
								kvState = NONQUOTEVALUE;
							}
							break;
					}				
					case ENDKV:
                                          {
                                            //std::cout << "ENDKV\n";

                                            chompEOL(infile);
                                            
                                            stripTrailing(key);
                                            organiser.foundData(key.c_str(), value.c_str(), comment.c_str());
                                            
                                            kvState = ENDKEYVALUE;
                                            break;
                                          }
                                        case LEFTARROW:
                                          {
                                            //std::cout << "LEFTARROW\n";

                                            int c = infile.peek();

                                            if(c == EOF || isEOL(c))
                                              {
                                                kvState = ENDSOURCEDEST;

                                                stripTrailing(value);

                                                size_t dotPos = value.find_first_of('.');

                                                if(dotPos != string::npos)
                                                  {
                                                    srcApp = value.substr(0,dotPos);
                                                    srcObj = value.substr(dotPos+1,value.length()-dotPos-1);

                                                    //std::cout << "srcApp: " << srcApp << " srcObj: " << srcObj << "\n";
                                                  }
                                                else
                                                  {
                                                    //std::cout << "srcObj: " << value << "\n";
                                                    
                                                    srcApp = "";
                                                    srcObj = value;
                                                  }
                                              }
                                            else if(c == ' ' || c == '\t')
                                              {
                                                // Discard whitespace
                                                infile.get();
                                              }
                                            else if(d_commentchar != 0 && c == d_commentchar)
                                              {
                                                // discard '#'
                                                infile.get();
                                                
                                                kvState = SDCOMMENT;
                                              }
                                            else 
                                              {
                                              kvState = GETSOURCE;
                                              }


                                            break;
                                          }
                                        case RIGHTARROW:
                                          {
                                            //std::cout << "RIGHTARROW\n";

                                            int c = infile.peek();

                                            if(c == EOF || isEOL(c))
                                              {
                                                kvState = ENDSOURCEDEST;

                                                stripTrailing(value);                                                

                                                size_t dotPos = value.find_first_of('.');

                                                if(dotPos != string::npos)
                                                  {
                                                    destApp = value.substr(0,dotPos);
                                                    destObj = value.substr(dotPos+1,value.length()-dotPos-1);
                                                  }
                                                else
                                                  {
                                                    destApp = "";
                                                    destObj = value;
                                                  }
                                              }
                                            else if(c == ' ' || c == '\t')
                                              {
                                                // Discard whitespace
                                                infile.get();
                                              }
                                            else if(d_commentchar != 0 && c == d_commentchar)
                                              {
                                                // discard '#'
                                                infile.get();
                                                
                                                kvState = SDCOMMENT;
                                              }
                                            else 
                                              {
                                              kvState = GETDEST;
                                              }

                                            break;
                                          }
                                        case GETSOURCE:
                                          {
                                            //std::cout << "GETSOURCE\n";

                                            int c = infile.peek();

                                            if(c == EOF || isEOL(c))
                                              {
                                                stripTrailing(value);

                                                kvState = ENDSOURCEDEST;

                                                size_t dotPos = value.find_first_of('.');

                                                if(dotPos != string::npos)
                                                  {
                                                    srcApp = value.substr(0,dotPos);
                                                    srcObj = value.substr(dotPos+1,value.length()-dotPos-1);

                                                    //std::cout << "srcApp: " << srcApp << " srcObj: " << srcObj << "\n";
                                                  }
                                                else
                                                  {
                                                    //std::cout << "srcObj: " << value << "\n";
                                                    
                                                    srcApp = "";
                                                    srcObj = value;
                                                  }
                                              }
                                            else if((d_commentchar != 0 && c == d_commentchar) || c == '[')
                                              {
                                                // discard '#' or '['
                                                //
                                                infile.get();
                                                
                                                stripTrailing(value);
                                                
                                                size_t dotPos = value.find_first_of('.');

                                                if(dotPos != string::npos)
                                                  {
                                                    srcApp = value.substr(0,dotPos);
                                                    srcObj = value.substr(dotPos+1,value.length()-dotPos-1);

                                                    //std::cout << "srcApp: " << srcApp << " srcObj: " << srcObj << "\n";
                                                  }
                                                else
                                                  {
                                                    //std::cout << "srcObj: " << value << "\n";
                                                    
                                                    srcApp = "";
                                                    srcObj = value;
                                                  }

                                                // Which one was it, comment about to start or width info?
                                                if(c == '[') 
                                                  kvState = GETWIDTH;
                                                else
                                                  kvState = SDCOMMENT;
                                              }
                                            else
                                              {
                                                // append to value (temp storage for 
                                                //
                                                value += infile.get();

                                                // LOOP
                                              }

                                            break;
                                          }
                                        case GETDEST:
                                          {

                                            //std::cout << "GETDEST\n";

                                            int c = infile.peek();

                                            if(c == EOF || isEOL(c))
                                              {
                                                stripTrailing(value);

                                                kvState = ENDSOURCEDEST;

                                                size_t dotPos = value.find_first_of('.');

                                                if(dotPos != string::npos)
                                                  {
                                                    destApp = value.substr(0,dotPos);
                                                    destObj = value.substr(dotPos+1,value.length()-dotPos-1);

                                                    //std::cout << "Found destApp: " << destApp << " destObj: " << destObj << "\n";
                                                                    
                                                  }
                                                else
                                                  {
                                                    destApp = "";
                                                    destObj = value;
                                                  }
                                              }
                                            else if((d_commentchar != 0 && c == d_commentchar) || c == '[')
                                              {
                                                // discard '#' or '['
                                                //
                                                infile.get();
                                                
                                                stripTrailing(value);
                                                
                                                size_t dotPos = value.find_first_of('.');

                                                if(dotPos != string::npos)
                                                  {
                                                    destApp = value.substr(0,dotPos);
                                                    destObj = value.substr(dotPos+1,value.length()-dotPos-1);

                                                    //std::cout << "Found destApp: " << destApp << " destObj: " << destObj << "\n";
                                                                    
                                                  }
                                                else
                                                  {
                                                    destApp = "";
                                                    destObj = value;
                                                  }

                                                // Which one was it, comment about to start or width info?
                                                if(c == '[') 
                                                  kvState = GETWIDTH;
                                                else
                                                  kvState = SDCOMMENT;
                                              }
                                            else
                                              {
                                                // append to value (temp storage for 
                                                //
                                                value += infile.get();

                                                // LOOP
                                              }

                                            break;
                                          }
                                        case GETWIDTH:
                                          {
                                            //std::cout << "GETWIDTH\n";

                                            int c = infile.peek();
                                            
                                            if(c == EOF || isEOL(c))
                                              {
                                                setError("901", "Partial width found. No terminating ']'.");
                                                kvState = KVERROR;
                                              }
                                            else if(d_commentchar != 0 && c == d_commentchar)
                                              {
                                                // discard '#'
                                                //
                                                infile.get();

                                                kvState = SDCOMMENT;
                                              }
                                            else if(c == ' ' || c == '\t')
                                              {
                                                // discard whitespace
                                                //
                                                infile.get();


                                                c = infile.peek();

                                                if('0' <= c && c <= '9' && width.length() > 0)
                                                  {
                                                    setError("902", "Two numbers detected in the width declaration.");
                                                    kvState = KVERROR;
                                                  }

                                                // LOOP
                                              }
                                            /* remedius
                                             * communication type (GETCOMMTYPE) and processing method(GETPROCMETHOD) parameters are optional.
                                             */
                                            else if(c == ',')
                                            {
                                            	stripTrailing(width);
                                            	infile.get(); // Throw away ,
                                            	kvState = GETCOMMTYPE;

                                            }
                                            else if(c == ']')
                                              {
                                                stripTrailing(width);
                                                infile.get(); // Throw away ]
                                                kvState = ENDSOURCEDEST;

                                                // Ignoring any comments that might be here...
                                              }
                                            else if('0' <= c && c <= '9')
                                              {
                                                width += infile.get();
                                              }
                                            else {
                                              setError("903", "Non integer number detected.");
                                              kvState = KVERROR;
                                            }
                                            break;
                                          }
                                          /* remedius */
                                        case GETCOMMTYPE:
                                        {
                                            int c = infile.peek();

                                            if(c == EOF || isEOL(c))
                                              {
                                                setError("901", "Partial communication type is found. No terminating ']'.");
                                                kvState = KVERROR;
                                              }
                                            else if(d_commentchar != 0 && c == d_commentchar)
                                              {
                                                // discard '#'
                                                //
                                                infile.get();

                                                kvState = SDCOMMENT;
                                              }
                                            else if(c == ' ' || c == '\t')
                                              {
                                                // discard whitespace
                                                //
                                                infile.get();

                                                // LOOP
                                              }
                                            else if(c == ']')
                                              {
                                                stripTrailing(commType);
                                                infile.get(); // Throw away ]
                                                kvState = ENDSOURCEDEST;

                                                // Ignoring any comments that might be here...
                                              }
                                            else if(c == ',')
                                            {
                                            	stripTrailing(commType);
                                            	infile.get(); // Throw away ,
                                            	kvState = GETPROCMETHOD;
                                            }
                                            else
                                              {
                                                commType += infile.get();
                                              }

                                            break;

                                        }
                                        /* remedius */
                                        case GETPROCMETHOD:
                                        {
                                        	int c = infile.peek();

                                        	if(c == EOF || isEOL(c))
                                        	{
                                        		setError("901", "Partial processing mathod is found. No terminating ']'.");
                                        		kvState = KVERROR;
                                        	}
                                        	else if(d_commentchar != 0 && c == d_commentchar)
                                        	{
                                        		// discard '#'
                                        		//
                                        		infile.get();

                                        		kvState = SDCOMMENT;
                                        	}
                                        	else if(c == ' ' || c == '\t')
                                        	{
                                        		// discard whitespace
                                        		//
                                        		infile.get();

                                        		// LOOP
                                        	}
                                        	else if(c == ']')
                                        	{
                                        		stripTrailing(commType);
                                        		infile.get(); // Throw away ]
                                        		kvState = ENDSOURCEDEST;

                                        		// Ignoring any comments that might be here...
                                        	}
                                        	else if(c != ',')
                                        	{
                                        		procMethod += infile.get();
                                        	}
                                        	else
                                        	{
                                        		setError("904", "Unexpected symbol.");
                                        		kvState = KVERROR;
                                        	}

                                        	break;

                                        }
                                        case SDCOMMENT:
                                          {
                                            //std::cout << "SDCOMMENT\n";

                                            int c = infile.peek();

                                            if(c == EOF || isEOL(c))
                                              {
                                                stripTrailing(comment);
                                                kvState = ENDSOURCEDEST;
                                              }
                                            else
                                              {
                                                // Append to comment
                                                //
                                                comment += infile.get();
                                                
                                                // LOOP	
                                              }
                                            break;
                                          }
                                        case ENDSOURCEDEST:
                                          {
                                            //std::cout << "ENDSOURCEDEST\n";

                                            chompEOL(infile);
                                            
                                            stripTrailing(srcApp);
                                            stripTrailing(srcObj);
                                            stripTrailing(destApp);
                                            stripTrailing(destObj);
                                            stripTrailing(width);
                                            /* remedius */
                                            stripTrailing(commType);
                                            /* remedius */
                                            stripTrailing(procMethod);
                                            /* remedius */
                                            organiser.foundSourceDest(srcApp.c_str(), srcObj.c_str(), destApp.c_str(), destObj.c_str(), width.c_str(), commType.c_str(), procMethod.c_str(), comment.c_str());
                                            
                                            kvState = ENDKEYVALUE;
                                            break;
                                          }


				} // end switch
			} // end while
		}
	}
	return true;
}

}} // end namespaces




