/** \file orcdchomp_kdata.h
 * \brief Interface to orcdchomp_kdata, a parser for sphere data provided
 *        with an OpenRAVE kinbody XML file.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012 Carnegie Mellon University */

/* requires:
 *  - openrave/openrave.h
 * */

namespace orcdchomp
{

struct sphereelem
{
   struct sphereelem * next;
   struct sphere * s;
};

struct sphere
{
   /* parsed from xml */
   char linkname[32];
   double pos[3];
   double radius;
};


/* the kinbody-attached data class */
class kdata : public OpenRAVE::XMLReadable
{
public:
   struct sphereelem * sphereelems;
   kdata();
   ~kdata();
};


/* the kdata-parser */
class kdata_parser : public OpenRAVE::BaseXMLReader
{
public:
   boost::shared_ptr<kdata> d;
   bool inside_spheres;

   kdata_parser(boost::shared_ptr<kdata> passed_d, const OpenRAVE::AttributesList& atts);
   virtual OpenRAVE::XMLReadablePtr GetReadable();
   virtual ProcessElement startElement(const std::string& name, const OpenRAVE::AttributesList& atts);
   virtual void characters(const std::string& ch);
   virtual bool endElement(const std::string& name);
};

} /* namespace orcdchomp */
