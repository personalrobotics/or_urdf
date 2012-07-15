/** \file orcdchomp_rdata.h
 * \brief Interface to orcdchomp_rdata, a parser for sphere data provided
 *        with an OpenRAVE robot XML file.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012 Carnegie Mellon University */

/* requires:
 *  - openrave/openrave.h
 * */

namespace orcdchomp
{

struct sphere
{
   struct sphere * next;
   /* parsed from xml */
   char linkname[32];
   double pos[3];
   double radius;
   /* solved for later */
   int linkindex; /* solved for on init */
};


/* the robot-attached data class */
class rdata : public OpenRAVE::XMLReadable
{
public:
   struct sphere * spheres;
   rdata();
   ~rdata();
};


/* the rdata-parser */
class rdata_parser : public OpenRAVE::BaseXMLReader
{
public:
   boost::shared_ptr<rdata> d;
   bool inside_spheres;

   rdata_parser(boost::shared_ptr<rdata> passed_d, const OpenRAVE::AttributesList& atts);
   virtual OpenRAVE::XMLReadablePtr GetReadable();
   virtual ProcessElement startElement(const std::string& name, const OpenRAVE::AttributesList& atts);
   virtual void characters(const std::string& ch);
   virtual bool endElement(const std::string& name);
};

} /* namespace orcdchomp */
